// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>

#include "auk/hal/hal.hpp"

namespace auk::sim
{

/// A simulated AUK, complete enough to exercise the whole control stack.
///
/// Time is virtual and advanced explicitly by the test or the demo driver, so a
/// scenario runs deterministically and as fast as the host will go -- a minute
/// of robot time in a few milliseconds. Nothing here sleeps.
///
/// The plant is deliberately simple but not trivial: wheels have a first-order
/// lag so commanded and measured speeds differ, and the battery sags under load.
/// That is enough to catch the class of bug where code assumes a command takes
/// effect instantly, which is most of them.
class SimBoard
{
public:
    SimBoard();

    hal::Board board() noexcept;

    /// Advance the simulation by one millisecond.
    void tick_ms() noexcept { advance(1); }

    /// Advance by `ms` milliseconds, integrating the plant each step.
    void advance(std::uint32_t ms) noexcept;

    // ---- scenario controls -------------------------------------------------

    void set_estop(bool asserted) noexcept { inputs_.estop_asserted = asserted; }
    void set_front_bumper(bool pressed) noexcept
    {
        inputs_.front_bumper_pressed = pressed;
    }
    void set_rear_bumper(bool pressed) noexcept { inputs_.rear_bumper_pressed = pressed; }
    void set_link_up(bool up) noexcept { link_up_ = up; }
    void set_battery_volts(float volts) noexcept { pack_volts_ = volts; }

    /// Queue text as if it had arrived from upstream.
    void send(const std::string& text);

    /// Everything the robot has transmitted since the last call.
    std::string take_output();

    // ---- observation -------------------------------------------------------

    /// True ground-truth pose of the simulated plant, which is what the
    /// odometry estimate should be compared against.
    float true_x_m() const noexcept { return true_x_m_; }
    float true_y_m() const noexcept { return true_y_m_; }
    float true_yaw_rad() const noexcept { return true_yaw_rad_; }

    bool torque_enabled() const noexcept { return !estop_engaged_; }
    bool lamp_on() const noexcept { return lamp_on_; }
    std::uint32_t watchdog_resets() const noexcept { return watchdog_resets_; }

private:
    class SimClock;
    class SimDrivetrain;
    class SimInputs;
    class SimPower;
    class SimLamp;
    class SimLink;
    class SimWatchdog;

    void integrate(float dt_s) noexcept;

    core::Millis now_ms_{0};

    // plant state
    control::WheelSpeeds commanded_{};
    control::WheelSpeeds actual_{};
    bool estop_engaged_{true};
    float true_x_m_{0.0F};
    float true_y_m_{0.0F};
    float true_yaw_rad_{0.0F};
    float track_width_m_{0.40F};

    /// First-order wheel response time constant, seconds.
    float wheel_tau_s_{0.08F};

    float pack_volts_{25.2F};
    float load_current_a_{0.0F};

    hal::DigitalInputs inputs_{};
    bool link_up_{true};
    bool lamp_on_{false};

    std::deque<std::uint8_t> rx_{};  ///< bytes waiting to reach the robot
    std::string tx_{};               ///< bytes the robot has sent

    core::Millis last_kick_ms_{0};
    std::uint32_t watchdog_timeout_ms_{0};
    bool watchdog_started_{false};
    std::uint32_t watchdog_resets_{0};

    // Owned implementations, defined in the .cpp.
    SimClock* clock_{nullptr};
    SimDrivetrain* drivetrain_{nullptr};
    SimInputs* input_panel_{nullptr};
    SimPower* power_{nullptr};
    SimLamp* lamp_{nullptr};
    SimLink* link_{nullptr};
    SimWatchdog* watchdog_{nullptr};

public:
    ~SimBoard();
    SimBoard(const SimBoard&) = delete;
    SimBoard& operator=(const SimBoard&) = delete;
};

}  // namespace auk::sim
