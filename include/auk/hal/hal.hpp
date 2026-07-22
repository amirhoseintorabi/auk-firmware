// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>

#include "auk/control/differential_drive.hpp"
#include "auk/core/deadline.hpp"

namespace auk::hal
{

/// The complete set of things the application needs from the hardware.
///
/// Everything below is an abstract interface with no platform headers behind it,
/// which is what lets the whole control stack -- scheduler, command bus,
/// kinematics, odometry, interlock, gauge -- build and run on a workstation
/// against the simulator in `sim/`. That is not a testing nicety: it means the
/// logic can be exercised against fault cases that are tedious or dangerous to
/// stage on a real robot, such as a bumper closing at the same instant the link
/// drops.
///
/// The interfaces are deliberately narrow. A HAL that exposes "the CAN bus"
/// invites the application to know about CAN; one that exposes "wheel speeds"
/// does not, and can be re-implemented over anything.

/// Monotonic millisecond clock. Must not jump backwards and must keep counting
/// across the 32-bit wrap; `core::elapsed_since` handles the wrap for callers.
class Clock
{
public:
    virtual ~Clock() = default;
    virtual core::Millis now_ms() const = 0;
};

/// The drivetrain.
class Drivetrain
{
public:
    virtual ~Drivetrain() = default;

    /// Command wheel speeds in m/s.
    virtual void set_wheel_speeds(const control::WheelSpeeds& speeds) = 0;

    /// Measured wheel speeds in m/s, from encoders.
    virtual control::WheelSpeeds measured_speeds() const = 0;

    /// Cut torque as directly as the hardware allows -- a driver disable or a
    /// contactor, not a zero setpoint.
    ///
    /// Separate from `set_wheel_speeds({0, 0})` on purpose. A zero setpoint asks
    /// a controller that may itself be wedged to please stop; this removes its
    /// ability to drive at all. When those two are the same call, the emergency
    /// stop is only as reliable as the thing it is trying to override.
    virtual void emergency_stop() = 0;

    /// Re-enable after an `emergency_stop`. Must be an explicit action: a robot
    /// that re-arms itself because a fresh setpoint arrived has no emergency
    /// stop, only a pause.
    virtual void release_emergency_stop() = 0;
};

/// Discrete inputs read as a set, so that the application sees one coherent
/// snapshot rather than values sampled at different moments.
struct DigitalInputs
{
    bool estop_asserted{false};
    bool front_bumper_pressed{false};
    bool rear_bumper_pressed{false};
};

class InputPanel
{
public:
    virtual ~InputPanel() = default;
    virtual DigitalInputs read() const = 0;
};

/// Pack measurements.
struct PowerSample
{
    float terminal_volts{0.0F};
    float current_a{0.0F};  ///< positive when discharging
};

class PowerMonitor
{
public:
    virtual ~PowerMonitor() = default;
    virtual PowerSample read() const = 0;
};

/// Work lamp or beacon.
class Lamp
{
public:
    virtual ~Lamp() = default;
    virtual void set(bool on) = 0;
};

/// Bytes in and out to whatever supervises the robot.
///
/// Framing and encoding are the application's problem, not the HAL's, so this
/// works equally over USB CDC, RS-485 or a TCP socket in the simulator.
class Link
{
public:
    virtual ~Link() = default;

    /// True while the peer is considered present.
    virtual bool is_up() const = 0;

    /// Non-blocking. Returns bytes read, never more than `capacity`.
    virtual std::size_t read(std::uint8_t* buffer, std::size_t capacity) = 0;

    /// Non-blocking. Returns bytes accepted, which may be fewer than `length`.
    virtual std::size_t write(const std::uint8_t* buffer, std::size_t length) = 0;
};

/// Hardware watchdog.
///
/// Present in the HAL rather than left to the port because a watchdog that is
/// optional is a watchdog that is absent. `app::Robot` kicks it from exactly one
/// place, at the end of a control iteration that completed, so a loop that
/// wedges part-way through stops kicking and the part resets.
///
/// Kicking it from a timer interrupt, which is the tempting shortcut, gives you
/// a robot that resets only when the whole chip is dead -- and cheerfully keeps
/// driving with a wedged control loop, which is the case you actually wanted to
/// catch.
class Watchdog
{
public:
    virtual ~Watchdog() = default;

    /// Arm with the given timeout. Must be called before the first `kick`.
    virtual void start(std::uint32_t timeout_ms) = 0;

    /// Restart the countdown.
    virtual void kick() = 0;
};

/// Everything bundled, so the application takes one parameter and every port
/// provides one struct.
struct Board
{
    Clock* clock{nullptr};
    Drivetrain* drivetrain{nullptr};
    InputPanel* inputs{nullptr};
    PowerMonitor* power{nullptr};
    Lamp* lamp{nullptr};
    Link* link{nullptr};
    Watchdog* watchdog{nullptr};

    /// True when every interface needed to run safely is present.
    bool complete() const noexcept
    {
        return clock != nullptr && drivetrain != nullptr && inputs != nullptr &&
               power != nullptr && lamp != nullptr && link != nullptr &&
               watchdog != nullptr;
    }
};

}  // namespace auk::hal
