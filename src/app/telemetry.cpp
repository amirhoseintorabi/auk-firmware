// SPDX-License-Identifier: MIT
#include "auk/app/telemetry.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace auk::app
{
namespace
{

/// Case-sensitive prefix match that also reports where the payload starts.
bool starts_with(const char* text, std::size_t length, const char* prefix,
                 std::size_t& offset) noexcept
{
    const std::size_t prefix_length = std::strlen(prefix);
    if (length < prefix_length || std::strncmp(text, prefix, prefix_length) != 0)
    {
        return false;
    }
    offset = prefix_length;
    return true;
}

/// Parse a float, rejecting anything that is not a complete, finite number.
///
/// `strtof` alone is too permissive for a command path: it happily accepts
/// "inf", "nan" and "12abc", and an infinity reaching the kinematics propagates
/// into the pose and never leaves.
bool parse_float(const char* text, std::size_t length, float& out) noexcept
{
    if (length == 0 || length >= 32)
    {
        return false;
    }

    char scratch[32];
    std::memcpy(scratch, text, length);
    scratch[length] = '\0';

    char* end = nullptr;
    const float value = std::strtof(scratch, &end);

    if (end == scratch || *end != '\0')
    {
        return false;  // empty, or trailing rubbish
    }
    if (!(value == value))
    {
        return false;  // NaN
    }
    if (value > 1e6F || value < -1e6F)
    {
        return false;  // infinity, or a magnitude no robot means
    }

    out = value;
    return true;
}

}  // namespace

std::size_t encode(const TelemetrySnapshot& s, char* out, std::size_t capacity) noexcept
{
    if (out == nullptr || capacity == 0)
    {
        return 0;
    }

    // snprintf truncates rather than overflowing and always terminates, which is
    // the property that matters here; its return value is the length it *wanted*
    // to write, so it is clamped below rather than trusted directly.
    const int written = std::snprintf(
        out, capacity,
        "S %lu %lu x=%.3f y=%.3f yaw=%.4f v=%.3f w=%.3f cv=%.3f cw=%.3f "
        "soc=%.3f volt=%.2f lamp=%d faults=0x%04x drop=%lu\n",
        static_cast<unsigned long>(s.timestamp_ms),
        static_cast<unsigned long>(s.sequence), static_cast<double>(s.pose.x_m),
        static_cast<double>(s.pose.y_m), static_cast<double>(s.pose.yaw_rad),
        static_cast<double>(s.pose.twist.linear_mps),
        static_cast<double>(s.pose.twist.angular_rps),
        static_cast<double>(s.commanded.linear_mps),
        static_cast<double>(s.commanded.angular_rps), static_cast<double>(s.battery_soc),
        static_cast<double>(s.battery_volts), s.lamp_on ? 1 : 0,
        static_cast<unsigned>(s.faults), static_cast<unsigned long>(s.commands_dropped));

    if (written < 0)
    {
        out[0] = '\0';
        return 0;
    }

    const std::size_t length = static_cast<std::size_t>(written);
    return (length >= capacity) ? capacity - 1 : length;
}

bool parse(const char* line, std::size_t length, ParsedCommand& out) noexcept
{
    if (line == nullptr)
    {
        return false;
    }

    // Trim trailing CR/LF and spaces so the protocol tolerates whichever line
    // ending the other end happens to use.
    while (length > 0 && (line[length - 1] == '\n' || line[length - 1] == '\r' ||
                          line[length - 1] == ' '))
    {
        --length;
    }
    if (length == 0)
    {
        return false;
    }

    out = ParsedCommand{};
    std::size_t offset = 0;

    // "V <linear> <angular>"
    if (starts_with(line, length, "V ", offset))
    {
        const char* rest = line + offset;
        const std::size_t rest_length = length - offset;

        const char* space = static_cast<const char*>(std::memchr(rest, ' ', rest_length));
        if (space == nullptr)
        {
            return false;
        }

        const std::size_t first_length = static_cast<std::size_t>(space - rest);
        if (!parse_float(rest, first_length, out.linear))
        {
            return false;
        }
        if (!parse_float(space + 1, rest_length - first_length - 1, out.angular))
        {
            return false;
        }

        out.kind = ParsedCommand::Kind::Velocity;
        return true;
    }

    // "L <0|1>"
    if (starts_with(line, length, "L ", offset))
    {
        float value = 0.0F;
        if (!parse_float(line + offset, length - offset, value))
        {
            return false;
        }
        out.kind = ParsedCommand::Kind::Lamp;
        out.flag = (value != 0.0F);
        return true;
    }

    // "E <0|1>"
    if (starts_with(line, length, "E ", offset))
    {
        float value = 0.0F;
        if (!parse_float(line + offset, length - offset, value))
        {
            return false;
        }
        out.kind = ParsedCommand::Kind::EmergencyStop;
        out.flag = (value != 0.0F);
        return true;
    }

    // "C" -- clear faults
    if (length == 1 && line[0] == 'C')
    {
        out.kind = ParsedCommand::Kind::ClearFaults;
        return true;
    }

    return false;
}

}  // namespace auk::app
