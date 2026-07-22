// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace auk::core
{

/// Outcome of an operation that can fail in a small, enumerable number of ways.
///
/// Firmware here never throws and never allocates, so errors travel as return
/// values. `Result::Ok` is deliberately non-zero: a struct that is
/// zero-initialised by accident then reads as `Unknown` rather than as success,
/// which turns a whole class of "forgot to set the status" bug into an obvious
/// one.
enum class Result : std::uint8_t
{
    Unknown = 0,  ///< Never assigned. Treat as failure.
    Ok,           ///< Completed.
    Empty,        ///< Nothing available (queue drained, no sample yet).
    Full,         ///< No space; the caller's data was not stored.
    OutOfRange,   ///< An argument fell outside the accepted domain.
    NotReady,     ///< Hardware or peripheral is not initialised yet.
    Timeout,      ///< A bounded wait expired.
};

/// True only for `Result::Ok`, so `if (ok(r))` cannot accidentally succeed on a
/// zero-initialised value.
constexpr bool ok(Result r) noexcept
{
    return r == Result::Ok;
}

/// Human-readable name, for logs and test failure messages.
constexpr const char* to_string(Result r) noexcept
{
    switch (r)
    {
        case Result::Ok:
            return "Ok";
        case Result::Empty:
            return "Empty";
        case Result::Full:
            return "Full";
        case Result::OutOfRange:
            return "OutOfRange";
        case Result::NotReady:
            return "NotReady";
        case Result::Timeout:
            return "Timeout";
        case Result::Unknown:
            break;
    }
    return "Unknown";
}

}  // namespace auk::core
