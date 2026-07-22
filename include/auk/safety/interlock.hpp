// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "auk/control/differential_drive.hpp"
#include "auk/core/deadline.hpp"

namespace auk::safety
{

/// Why motion is currently restricted. A bit field, because several can hold at
/// once and an operator staring at a stationary robot needs to know about all of
/// them, not just whichever the code happened to test first.
///
/// Sixteen bits rather than the eight the current seven flags would fit in.
/// That is deliberate on two counts: the telemetry format publishes this as four
/// hex digits, so narrowing it would be a wire-format change, and a fault set is
/// the thing most likely to grow as a machine acquires sensors. One spare byte
/// is a cheap price for not having to revisit either.
// NOLINTNEXTLINE(performance-enum-size)
enum class FaultFlags : std::uint16_t
{
    None = 0,
    EmergencyStop = 1U << 0,  ///< e-stop asserted, from any source
    FrontBumper = 1U << 1,
    RearBumper = 1U << 2,
    LinkLost = 1U << 3,      ///< no telemetry link
    CommandStale = 1U << 4,  ///< link is up but setpoints stopped arriving
    BatteryCritical = 1U << 5,
    SafetyQueueLost = 1U << 6,  ///< the high-priority command lane overflowed
};

constexpr FaultFlags operator|(FaultFlags a, FaultFlags b) noexcept
{
    return static_cast<FaultFlags>(static_cast<std::uint16_t>(a) |
                                   static_cast<std::uint16_t>(b));
}

constexpr FaultFlags& operator|=(FaultFlags& a, FaultFlags b) noexcept
{
    a = a | b;
    return a;
}

constexpr bool any(FaultFlags value) noexcept
{
    return static_cast<std::uint16_t>(value) != 0;
}

constexpr bool has(FaultFlags value, FaultFlags flag) noexcept
{
    return (static_cast<std::uint16_t>(value) & static_cast<std::uint16_t>(flag)) != 0;
}

/// Everything the interlock needs to decide what motion to permit.
///
/// Passed in as one struct rather than read from module globals so that the
/// decision is a pure function of its inputs. That is what makes the truth table
/// below testable: every combination that matters can be constructed directly,
/// including the ones that are difficult to stage on real hardware.
struct SafetyInputs
{
    bool estop_asserted{false};
    bool front_bumper_pressed{false};
    bool rear_bumper_pressed{false};
    bool link_up{false};
    bool battery_critical{false};
    bool safety_queue_overflowed{false};

    /// When a velocity setpoint last arrived.
    core::Millis last_command_ms{0};
};

/// Decides what the robot is allowed to do, and is the only thing that does.
///
/// Two design rules, both learned the expensive way:
///
///  1. **Absence of permission is not permission.** The interlock starts in a
///     faulted state and stays there until it has positively seen a healthy
///     input set. A robot that powers up moving because nothing had yet told it
///     not to is a robot that hurts somebody during a brownout.
///
///  2. **A bumper stops the robot, not just its lamp.** Blocking motion *into* a
///     triggered bumper while still allowing escape is the whole job. It is also
///     the interlock most likely to get commented out during bring-up, because
///     an over-sensitive bumper is maddening -- which is exactly why it lives
///     here, behind a truth table with tests on it, rather than inline in a
///     control loop where disabling it is a one-line edit nobody reviews.
class Interlock
{
public:
    /// A command with no matching setpoint for this long is treated as stale and
    /// motion is cut. This is the backstop for a controller that has crashed
    /// while its transport stays nominally connected -- the link looks fine, the
    /// setpoints simply stop, and without this the robot keeps driving on the
    /// last thing it was told.
    static constexpr std::uint32_t kCommandTimeoutMs = 300;

    /// Evaluate the current inputs. Call once per control period, before using
    /// `permit`.
    void update(const SafetyInputs& inputs, core::Millis now) noexcept;

    /// Restrict a requested twist to what is currently allowed.
    ///
    /// Returns the zero twist whenever any fault other than a bumper is active.
    /// For a bumper, only the component driving into it is removed, so the
    /// operator can always reverse out of a collision -- a robot that has to be
    /// carried off its own bumper is worse than one that will not drive into it.
    control::Twist permit(const control::Twist& request) const noexcept;

    FaultFlags faults() const noexcept { return faults_; }

    /// True when motion is permitted at all.
    bool motion_allowed() const noexcept { return !any(blocking_faults()); }

    /// Faults that stop the robot outright, excluding the directional bumper
    /// restrictions, which only remove one component.
    FaultFlags blocking_faults() const noexcept;

private:
    // Faulted until proven otherwise -- see rule 1 above.
    FaultFlags faults_{FaultFlags::EmergencyStop | FaultFlags::LinkLost};
};

}  // namespace auk::safety
