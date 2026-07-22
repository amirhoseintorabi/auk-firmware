// SPDX-License-Identifier: MIT
#include "sim/sim_board.hpp"

#include <algorithm>
#include <cmath>

#include "auk/control/odometry.hpp"

namespace auk::sim
{

class SimBoard::SimClock final : public hal::Clock
{
public:
    explicit SimClock(SimBoard& owner) : owner_{owner} {}
    core::Millis now_ms() const override { return owner_.now_ms_; }

private:
    SimBoard& owner_;
};

class SimBoard::SimDrivetrain final : public hal::Drivetrain
{
public:
    explicit SimDrivetrain(SimBoard& owner) : owner_{owner} {}

    void set_wheel_speeds(const control::WheelSpeeds& speeds) override
    {
        // A setpoint arriving while torque is disabled is recorded but has no
        // effect, which mirrors a real driver holding its output stage off.
        owner_.commanded_ = speeds;
    }

    control::WheelSpeeds measured_speeds() const override { return owner_.actual_; }

    void emergency_stop() override
    {
        owner_.estop_engaged_ = true;
        owner_.commanded_ = control::WheelSpeeds{};
    }

    void release_emergency_stop() override { owner_.estop_engaged_ = false; }

private:
    SimBoard& owner_;
};

class SimBoard::SimInputs final : public hal::InputPanel
{
public:
    explicit SimInputs(SimBoard& owner) : owner_{owner} {}
    hal::DigitalInputs read() const override { return owner_.inputs_; }

private:
    SimBoard& owner_;
};

class SimBoard::SimPower final : public hal::PowerMonitor
{
public:
    explicit SimPower(SimBoard& owner) : owner_{owner} {}

    hal::PowerSample read() const override
    {
        hal::PowerSample sample{};
        sample.current_a = owner_.load_current_a_;
        // Terminal voltage sags under load by I*R, so anything that reads it
        // without compensating will see the pack "empty" whenever the robot
        // accelerates.
        sample.terminal_volts = owner_.pack_volts_ - (owner_.load_current_a_ * 0.02F);
        return sample;
    }

private:
    SimBoard& owner_;
};

class SimBoard::SimLamp final : public hal::Lamp
{
public:
    explicit SimLamp(SimBoard& owner) : owner_{owner} {}
    void set(bool on) override { owner_.lamp_on_ = on; }

private:
    SimBoard& owner_;
};

class SimBoard::SimLink final : public hal::Link
{
public:
    explicit SimLink(SimBoard& owner) : owner_{owner} {}

    bool is_up() const override { return owner_.link_up_; }

    std::size_t read(std::uint8_t* buffer, std::size_t capacity) override
    {
        if (!owner_.link_up_)
        {
            return 0;
        }

        std::size_t count = 0;
        while (count < capacity && !owner_.rx_.empty())
        {
            buffer[count++] = owner_.rx_.front();
            owner_.rx_.pop_front();
        }
        return count;
    }

    std::size_t write(const std::uint8_t* buffer, std::size_t length) override
    {
        if (!owner_.link_up_)
        {
            return 0;  // a down link accepts nothing, as a real one would
        }
        owner_.tx_.append(reinterpret_cast<const char*>(buffer), length);
        return length;
    }

private:
    SimBoard& owner_;
};

class SimBoard::SimWatchdog final : public hal::Watchdog
{
public:
    explicit SimWatchdog(SimBoard& owner) : owner_{owner} {}

    void start(std::uint32_t timeout_ms) override
    {
        owner_.watchdog_timeout_ms_ = timeout_ms;
        owner_.watchdog_started_ = true;
        owner_.last_kick_ms_ = owner_.now_ms_;
    }

    void kick() override { owner_.last_kick_ms_ = owner_.now_ms_; }

private:
    SimBoard& owner_;
};

SimBoard::SimBoard()
    : clock_{new SimClock{*this}},
      drivetrain_{new SimDrivetrain{*this}},
      input_panel_{new SimInputs{*this}},
      power_{new SimPower{*this}},
      lamp_{new SimLamp{*this}},
      link_{new SimLink{*this}},
      watchdog_{new SimWatchdog{*this}}
{
}

SimBoard::~SimBoard()
{
    delete clock_;
    delete drivetrain_;
    delete input_panel_;
    delete power_;
    delete lamp_;
    delete link_;
    delete watchdog_;
}

hal::Board SimBoard::board() noexcept
{
    hal::Board b{};
    b.clock = clock_;
    b.drivetrain = drivetrain_;
    b.inputs = input_panel_;
    b.power = power_;
    b.lamp = lamp_;
    b.link = link_;
    b.watchdog = watchdog_;
    return b;
}

void SimBoard::send(const std::string& text)
{
    for (char c : text)
    {
        rx_.push_back(static_cast<std::uint8_t>(c));
    }
}

std::string SimBoard::take_output()
{
    std::string out;
    out.swap(tx_);
    return out;
}

void SimBoard::integrate(float dt_s) noexcept
{
    // Torque disabled means the wheels coast to a stop rather than snapping to
    // zero, which is what actually happens and is what makes "we cut torque, is
    // it stopped yet" a question worth asking in a test.
    const control::WheelSpeeds target =
        estop_engaged_ ? control::WheelSpeeds{} : commanded_;

    // First-order lag towards the target.
    const float alpha =
        (wheel_tau_s_ > 0.0F) ? std::min(1.0F, dt_s / wheel_tau_s_) : 1.0F;
    actual_.left_mps += (target.left_mps - actual_.left_mps) * alpha;
    actual_.right_mps += (target.right_mps - actual_.right_mps) * alpha;

    const float v = 0.5F * (actual_.left_mps + actual_.right_mps);
    const float w = (actual_.right_mps - actual_.left_mps) / track_width_m_;

    true_x_m_ += v * std::cos(true_yaw_rad_) * dt_s;
    true_y_m_ += v * std::sin(true_yaw_rad_) * dt_s;
    true_yaw_rad_ = control::wrap_angle(true_yaw_rad_ + w * dt_s);

    // Crude but adequate load model: current rises with wheel effort, plus a
    // quiescent draw for the electronics.
    load_current_a_ =
        1.0F + 6.0F * (std::fabs(actual_.left_mps) + std::fabs(actual_.right_mps));
}

void SimBoard::advance(std::uint32_t ms) noexcept
{
    for (std::uint32_t i = 0; i < ms; ++i)
    {
        ++now_ms_;
        integrate(0.001F);

        if (watchdog_started_ &&
            core::elapsed_since(last_kick_ms_, now_ms_) > watchdog_timeout_ms_)
        {
            // A real part would reset here. The simulator records it and
            // re-arms, so a test can assert that the watchdog *would* have
            // fired without the harness disappearing underneath it.
            ++watchdog_resets_;
            last_kick_ms_ = now_ms_;
        }
    }
}

}  // namespace auk::sim
