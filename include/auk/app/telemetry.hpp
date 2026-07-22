// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>

#include "auk/control/odometry.hpp"
#include "auk/safety/interlock.hpp"

namespace auk::app
{

/// One snapshot of everything the robot reports upstream.
///
/// Assembled once per publish period from the modules that own each field, then
/// serialised in one go. Building it as a value rather than publishing field by
/// field means a consumer always receives a self-consistent picture: a pose and
/// the fault state that applied at that pose, not a pose from this cycle beside
/// a fault flag from the last one.
struct TelemetrySnapshot
{
    core::Millis timestamp_ms{0};
    std::uint32_t sequence{0};  ///< increments per publish; gaps mean drops

    control::Pose2D pose{};
    control::Twist commanded{};  ///< after interlock and slew limiting

    float battery_soc{1.0F};
    float battery_volts{0.0F};
    bool lamp_on{false};

    safety::FaultFlags faults{safety::FaultFlags::None};
    std::uint32_t commands_dropped{0};
};

/// Line-oriented text encoding of a snapshot.
///
/// Text rather than a packed binary struct, deliberately. This link carries a
/// few hundred bytes at 20 Hz, so the bandwidth saving would be irrelevant,
/// while the cost of a binary format is real: silent breakage whenever the two
/// ends are built from different headers, and a wire you cannot debug without
/// custom tooling. A format you can read with a serial terminal is worth a great
/// deal at three in the morning.
///
/// Returns the number of bytes written, never exceeding `capacity`, and always
/// null-terminates when `capacity > 0`.
std::size_t encode(const TelemetrySnapshot& snapshot, char* out,
                   std::size_t capacity) noexcept;

/// One parsed command from upstream.
struct ParsedCommand
{
    enum class Kind : std::uint8_t
    {
        None = 0,
        Velocity,       ///< `linear` m/s, `angular` rad/s
        Lamp,           ///< `flag`
        EmergencyStop,  ///< `flag`
        ClearFaults,
    };

    Kind kind{Kind::None};
    float linear{0.0F};
    float angular{0.0F};
    bool flag{false};
};

/// Parse one complete line of the uplink protocol.
///
/// Returns false for anything not understood, including partial and malformed
/// input. A parser on a robot's command path is an attack surface and a fault
/// source, so this one rejects rather than guesses: an unrecognised line moves
/// nothing.
bool parse(const char* line, std::size_t length, ParsedCommand& out) noexcept;

}  // namespace auk::app
