// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>

#include "auk/app/telemetry.hpp"
#include "auk/control/differential_drive.hpp"
#include "auk/control/odometry.hpp"
#include "auk/control/slew_limiter.hpp"
#include "auk/core/command.hpp"
#include "auk/core/deadline.hpp"
#include "auk/hal/hal.hpp"
#include "auk/power/battery_gauge.hpp"
#include "auk/safety/interlock.hpp"

namespace auk::app
{

/// Tuning that a port is expected to set for its own machine.
struct RobotConfig
{
    control::DriveGeometry geometry{};
    power::BatteryConfig battery{};

    /// Longitudinal acceleration and deceleration limits, m/s^2.
    float accel_limit{0.8F};
    float decel_limit{2.0F};

    /// Yaw acceleration and deceleration limits, rad/s^2.
    float angular_accel_limit{2.5F};
    float angular_decel_limit{5.0F};

    /// Control period. Everything downstream assumes this is the interval
    /// between `step` calls doing work, so the odometry integration step and the
    /// slew limits are all expressed against it.
    std::uint32_t control_period_ms{10};

    /// Telemetry publish period.
    std::uint32_t telemetry_period_ms{50};

    /// Watchdog timeout. Comfortably longer than one control period so that
    /// ordinary jitter does not reset the robot, and short enough that a wedged
    /// loop is caught within a few control cycles.
    std::uint32_t watchdog_timeout_ms{100};
};

/// The robot application: one cooperative loop over a fixed set of modules.
///
/// There is no RTOS. Everything runs in one context, each activity gated by its
/// own `Deadline`, so there are no priorities to invert, no stacks to size, and
/// no locks. The cost is that every activity must be non-blocking and short --
/// which is a constraint worth accepting, because on a machine this size it is
/// also the thing that makes the timing analysable by reading the code.
///
/// The rule the whole design rests on: **nothing outside `dispatch` changes what
/// the robot does.** Producers post commands. `step` drains them, applies the
/// interlock, and commands the drivetrain. When somebody asks "what could make
/// this thing move", the answer is one function.
class Robot
{
public:
    Robot(const hal::Board& board, const RobotConfig& config) noexcept;

    /// Bring up hardware and arm the watchdog. Returns false if the board is
    /// incomplete, in which case `step` will do nothing -- refusing to run is
    /// the only safe response to a half-populated HAL.
    bool begin() noexcept;

    /// Run one iteration. Call as fast as convenient; internal deadlines decide
    /// what actually executes. Returns false if `begin` did not succeed.
    bool step() noexcept;

    /// Post a command from outside the loop -- a link parser, a button ISR.
    /// Safe to call from an interrupt for the high-priority lane.
    core::Result post(const core::Command& command) noexcept
    {
        return bus_.post(command);
    }

    const control::Pose2D& pose() const noexcept { return odometry_.pose(); }
    safety::FaultFlags faults() const noexcept { return interlock_.faults(); }
    const TelemetrySnapshot& last_telemetry() const noexcept { return snapshot_; }

private:
    void dispatch_commands(core::Millis now) noexcept;
    void apply(const core::Command& command, core::Millis now) noexcept;
    void run_control(core::Millis now) noexcept;
    void service_link(core::Millis now) noexcept;
    void publish(core::Millis now) noexcept;

    hal::Board board_;
    RobotConfig config_;

    core::CommandBus bus_{};
    control::DifferentialDrive drive_;
    control::Odometry odometry_;
    control::SlewLimiter linear_ramp_;
    control::SlewLimiter angular_ramp_;
    power::BatteryGauge gauge_;
    safety::Interlock interlock_{};

    core::Deadline control_due_{};
    core::Deadline telemetry_due_{};

    control::Twist requested_{};  ///< newest accepted setpoint
    control::Twist commanded_{};  ///< what actually went to the wheels
    core::Millis last_velocity_ms_{0};

    bool lamp_on_{false};
    bool estop_requested_{false};
    bool estop_engaged_{false};
    bool running_{false};

    TelemetrySnapshot snapshot_{};
    std::uint32_t sequence_{0};

    /// Reassembly buffer for the uplink, which arrives in arbitrary chunks.
    static constexpr std::size_t kLineCapacity = 128;
    char line_[kLineCapacity]{};
    std::size_t line_length_{0};
};

}  // namespace auk::app
