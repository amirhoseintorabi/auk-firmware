// SPDX-License-Identifier: MIT
#include "auk/app/robot.hpp"

#include <cstring>

namespace auk::app
{

Robot::Robot(const hal::Board& board, const RobotConfig& config) noexcept
    : board_{board},
      config_{config},
      drive_{config.geometry},
      odometry_{drive_},
      linear_ramp_{config.accel_limit, config.decel_limit},
      angular_ramp_{config.angular_accel_limit, config.angular_decel_limit},
      gauge_{config.battery}
{
}

bool Robot::begin() noexcept
{
    if (!board_.complete())
    {
        // A missing interface means some part of the machine is unreachable, and
        // we cannot tell which. Running with a null drivetrain or a null
        // watchdog is strictly worse than not running.
        return false;
    }

    const core::Millis now = board_.clock->now_ms();

    control_due_ = core::Deadline{now, config_.control_period_ms};
    telemetry_due_ = core::Deadline{now, config_.telemetry_period_ms};

    // Nothing has been heard from upstream yet, so the command timeout is
    // already expired and the interlock starts faulted. That is intentional: the
    // robot must be told to move before it moves.
    last_velocity_ms_ = now - safety::Interlock::kCommandTimeoutMs - 1;

    board_.drivetrain->emergency_stop();
    estop_engaged_ = true;

    board_.lamp->set(false);
    board_.watchdog->start(config_.watchdog_timeout_ms);

    running_ = true;
    return true;
}

bool Robot::step() noexcept
{
    if (!running_)
    {
        return false;
    }

    const core::Millis now = board_.clock->now_ms();

    // The link is serviced every iteration rather than on a deadline, because
    // input that is not drained promptly overflows a receive buffer somewhere,
    // and a dropped byte costs a whole command line.
    service_link(now);

    if (control_due_.expired(now))
    {
        control_due_.arm_periodic(now);
        dispatch_commands(now);
        run_control(now);
    }

    if (telemetry_due_.expired(now))
    {
        telemetry_due_.arm_periodic(now);
        publish(now);
    }

    // Kicked last, and only here. Reaching this line means a whole iteration
    // completed; anything that wedges earlier stops kicking and the watchdog
    // resets the part. A kick at the top of the loop, or from a timer ISR, would
    // keep the robot alive while its control loop was dead.
    board_.watchdog->kick();
    return true;
}

void Robot::dispatch_commands(core::Millis now) noexcept
{
    core::Command command{};

    // Drain the safety lane completely. Every element in it is a discrete event
    // -- a bumper edge, an e-stop press -- and handling only one per cycle would
    // let a press and its matching release sit in the queue across cycles, which
    // is how a robot ends up believing a button is still held.
    while (core::ok(bus_.take(core::Priority::High, command)))
    {
        apply(command, now);
    }

    // One each from the lower lanes. These are level-like rather than edge-like:
    // the newest velocity setpoint supersedes the last, so there is nothing to
    // gain from draining, and a bounded amount of work per cycle keeps the loop
    // period predictable.
    if (core::ok(bus_.take(core::Priority::Normal, command)))
    {
        apply(command, now);
    }
    if (core::ok(bus_.take(core::Priority::Low, command)))
    {
        apply(command, now);
    }
}

void Robot::apply(const core::Command& command, core::Millis now) noexcept
{
    switch (command.type)
    {
        case core::CommandType::SetVelocity:
        {
            // A setpoint that sat in a queue longer than the timeout is worse
            // than no setpoint: acting on it means moving to a demand the
            // operator has already replaced. Safety-lane commands are exempt --
            // an e-stop is never stale.
            if (core::elapsed_since(command.timestamp, now) >
                safety::Interlock::kCommandTimeoutMs)
            {
                break;
            }

            requested_.linear_mps = command.values[0];
            requested_.angular_rps = command.values[1];
            last_velocity_ms_ = command.timestamp;
            break;
        }

        case core::CommandType::SetLamp:
            lamp_on_ = (command.values[0] != 0.0F);
            board_.lamp->set(lamp_on_);
            break;

        case core::CommandType::SetEmergencyStop:
            estop_requested_ = (command.values[0] != 0.0F);
            break;

        // Both of these are deliberately empty, and share a case for that
        // reason: an accidentally-duplicated branch and an intentional one look
        // identical to a reader, and to a static analyser.
        //
        // The interlock re-reads both the bumpers and the link state from
        // hardware on every cycle, so there is no edge to latch here. These
        // commands exist so that a change wakes the control loop immediately
        // and shows up in a command trace, rather than only being noticed at
        // the next poll.
        case core::CommandType::BumperChanged:
        case core::CommandType::LinkStateChanged:
            break;

        case core::CommandType::ClearFaults:
            // Clears only what the operator is allowed to clear. The e-stop is
            // not in that set: it is released by the physical device, and a
            // software command that could override it would make the button
            // advisory.
            estop_requested_ = false;
            break;

        case core::CommandType::None:
            break;
    }
}

void Robot::run_control(core::Millis now) noexcept
{
    const hal::DigitalInputs inputs = board_.inputs->read();
    const hal::PowerSample power = board_.power->read();

    gauge_.update(power.terminal_volts, power.current_a);

    safety::SafetyInputs safety_inputs{};
    safety_inputs.estop_asserted = inputs.estop_asserted || estop_requested_;
    safety_inputs.front_bumper_pressed = inputs.front_bumper_pressed;
    safety_inputs.rear_bumper_pressed = inputs.rear_bumper_pressed;
    safety_inputs.link_up = board_.link->is_up();
    safety_inputs.battery_critical = gauge_.has_reading() && gauge_.is_critical();
    safety_inputs.safety_queue_overflowed = bus_.safety_lane_overflowed();
    safety_inputs.last_command_ms = last_velocity_ms_;

    interlock_.update(safety_inputs, now);

    const control::Twist allowed = interlock_.permit(requested_);
    const float dt_s = static_cast<float>(config_.control_period_ms) / 1000.0F;

    if (interlock_.motion_allowed())
    {
        if (estop_engaged_)
        {
            // Re-arming is an explicit transition, taken only once the interlock
            // is satisfied. The ramps are reset first so the drivetrain does not
            // receive a large step left over from before the stop.
            linear_ramp_.reset(0.0F);
            angular_ramp_.reset(0.0F);
            board_.drivetrain->release_emergency_stop();
            estop_engaged_ = false;
        }

        commanded_.linear_mps = linear_ramp_.update(allowed.linear_mps, dt_s);
        commanded_.angular_rps = angular_ramp_.update(allowed.angular_rps, dt_s);
        board_.drivetrain->set_wheel_speeds(drive_.to_wheels(commanded_));
    }
    else
    {
        // Cut torque at the hardware, and zero the ramps so that whenever motion
        // is permitted again it starts from a standstill rather than resuming
        // the demand that was in flight when the fault appeared.
        if (!estop_engaged_)
        {
            board_.drivetrain->emergency_stop();
            estop_engaged_ = true;
        }
        linear_ramp_.reset(0.0F);
        angular_ramp_.reset(0.0F);
        commanded_ = control::Twist{};
    }

    // Odometry integrates measured speeds, not commanded ones, so a stalled or
    // slipping wheel shows up as the robot not moving instead of being invisible.
    odometry_.update(board_.drivetrain->measured_speeds(), dt_s);
}

void Robot::service_link(core::Millis now) noexcept
{
    std::uint8_t chunk[64];
    const std::size_t count = board_.link->read(chunk, sizeof(chunk));

    for (std::size_t i = 0; i < count; ++i)
    {
        const char c = static_cast<char>(chunk[i]);

        if (c == '\n' || c == '\r')
        {
            if (line_length_ > 0)
            {
                ParsedCommand parsed{};
                if (parse(line_, line_length_, parsed))
                {
                    core::Command command{};
                    command.timestamp = now;

                    switch (parsed.kind)
                    {
                        case ParsedCommand::Kind::Velocity:
                            command.type = core::CommandType::SetVelocity;
                            command.priority = core::Priority::Normal;
                            command.values[0] = parsed.linear;
                            command.values[1] = parsed.angular;
                            break;

                        case ParsedCommand::Kind::Lamp:
                            command.type = core::CommandType::SetLamp;
                            command.priority = core::Priority::Low;
                            command.values[0] = parsed.flag ? 1.0F : 0.0F;
                            break;

                        case ParsedCommand::Kind::EmergencyStop:
                            command.type = core::CommandType::SetEmergencyStop;
                            command.priority = core::Priority::High;
                            command.values[0] = parsed.flag ? 1.0F : 0.0F;
                            break;

                        case ParsedCommand::Kind::ClearFaults:
                            command.type = core::CommandType::ClearFaults;
                            command.priority = core::Priority::Low;
                            break;

                        case ParsedCommand::Kind::None:
                            break;
                    }

                    if (command.type != core::CommandType::None)
                    {
                        (void) bus_.post(command);
                    }
                }
                line_length_ = 0;
            }
            continue;
        }

        if (line_length_ + 1 < kLineCapacity)
        {
            line_[line_length_++] = c;
        }
        else
        {
            // Over-length input is discarded whole rather than truncated. A
            // truncated line can still parse -- "V 1.0 2.5" cut short becomes
            // "V 1.0 2", a valid and quite different command -- so the only safe
            // response is to drop everything up to the next terminator.
            line_length_ = 0;
        }
    }
}

void Robot::publish(core::Millis now) noexcept
{
    snapshot_.timestamp_ms = now;
    snapshot_.sequence = ++sequence_;
    snapshot_.pose = odometry_.pose();
    snapshot_.commanded = commanded_;
    snapshot_.battery_soc = gauge_.state_of_charge();
    snapshot_.battery_volts = gauge_.open_circuit_volts();
    snapshot_.lamp_on = lamp_on_;
    snapshot_.faults = interlock_.faults();
    snapshot_.commands_dropped = bus_.dropped();

    char buffer[256];
    const std::size_t length = encode(snapshot_, buffer, sizeof(buffer));
    if (length > 0)
    {
        // Best effort. If the link cannot take it, the frame is dropped rather
        // than retried: telemetry is a stream of snapshots, the next one is
        // along in a few tens of milliseconds, and blocking the control loop to
        // deliver a stale one would be a poor trade. The sequence number lets
        // the far end see that it happened.
        (void) board_.link->write(reinterpret_cast<const std::uint8_t*>(buffer), length);
    }
}

}  // namespace auk::app
