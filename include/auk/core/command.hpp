// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "auk/core/deadline.hpp"
#include "auk/core/result.hpp"
#include "auk/core/ring_buffer.hpp"

namespace auk::core
{

/// What a command asks the robot to do.
///
/// Producers (the link parser, the bumper driver, the local button panel) never
/// act on the robot directly; they post one of these and return. Everything that
/// actually changes state happens in one place, on one thread, in
/// `app::Robot::dispatch`. That is the whole point of the bus: with a dozen
/// producers and one consumer there is exactly one function to read when you
/// want to know what can move the robot.
enum class CommandType : std::uint8_t
{
    None = 0,

    /// Requested body velocity. `values[0]` m/s forward, `values[1]` rad/s yaw.
    SetVelocity,

    /// Work lamp. `values[0]` is 0 for off, non-zero for on.
    SetLamp,

    /// Emergency stop asserted or cleared by an external source.
    /// `values[0]` non-zero to assert.
    SetEmergencyStop,

    /// A bumper changed state. `values[0]` is the bumper index,
    /// `values[1]` non-zero if pressed.
    BumperChanged,

    /// The telemetry link came up or went down. `values[0]` non-zero for up.
    LinkStateChanged,

    /// Clear latched faults, if the conditions that caused them have cleared.
    ClearFaults,
};

/// Dispatch order. Within one lane, commands are handled oldest-first.
enum class Priority : std::uint8_t
{
    /// Safety-relevant, must not be dropped: e-stop, bumpers, link loss.
    High = 0,
    /// Motion setpoints. Only the newest matters.
    Normal,
    /// Housekeeping: lamps, fault clearing.
    Low,

    Count,
};

struct Command
{
    CommandType type{CommandType::None};
    Priority priority{Priority::Low};

    /// When the command was created, not when it was dispatched. The consumer
    /// uses this to discard stale motion requests rather than acting on a
    /// setpoint that has been sitting in a queue behind a burst of other work.
    Millis timestamp{0};

    /// Payload. Meaning depends on `type`; see the enumerators above.
    ///
    /// Value-initialised so that a producer filling in one field leaves the
    /// others at zero rather than at whatever the stack happened to hold. An
    /// uninitialised payload here is a command that means something different
    /// from what its author wrote, which is a genuinely unpleasant bug to chase.
    float values[2]{0.0F, 0.0F};
};

/// Three-lane priority queue in front of the robot's single dispatch loop.
///
/// Capacities are chosen from what each lane is for, not by picking a round
/// number:
///
///  - `High` is deep enough to absorb every discrete safety event that could
///    plausibly land inside one control period, and rejects rather than
///    overwrites: dropping an e-stop because a bumper burst filled the queue
///    would be exactly the wrong trade.
///  - `Normal` holds one element and overwrites. A velocity setpoint that has
///    been superseded has no value, and queueing them would let the robot work
///    through a backlog of stale motion after a stall.
///  - `Low` is small and overwrites, because nothing in it is urgent.
class CommandBus
{
public:
    /// Post a command. Returns `Result::Full` only for the `High` lane, which
    /// is the only one that rejects.
    Result post(const Command& command) noexcept;

    /// Take the oldest command from `lane`, or `Result::Empty`.
    Result take(Priority lane, Command& out) noexcept;

    /// Total commands dropped from the overwriting lanes since boot. Non-zero is
    /// not itself a fault -- the `Normal` lane is *expected* to drop, since that
    /// is what "keep only the newest setpoint" means -- but a sharp rise in the
    /// `Low` lane means housekeeping is not being serviced.
    std::uint32_t dropped() const noexcept;

    /// True if the high-priority lane has ever rejected a command. This one *is*
    /// a fault: it means a safety event was lost.
    bool safety_lane_overflowed() const noexcept { return high_overflowed_; }

private:
    static constexpr std::size_t kHighCapacity = 16;
    static constexpr std::size_t kNormalCapacity = 1;
    static constexpr std::size_t kLowCapacity = 8;

    RingBuffer<Command, kHighCapacity, OverflowPolicy::Reject> high_{};
    RingBuffer<Command, kNormalCapacity, OverflowPolicy::DropOldest> normal_{};
    RingBuffer<Command, kLowCapacity, OverflowPolicy::DropOldest> low_{};
    bool high_overflowed_{false};
};

}  // namespace auk::core
