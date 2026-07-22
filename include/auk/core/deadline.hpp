// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

namespace auk::core
{

/// Milliseconds since boot. Wraps every 49.7 days.
using Millis = std::uint32_t;

/// Elapsed time between two millisecond timestamps, correct across the 32-bit
/// wrap.
///
/// The subtraction is done in unsigned arithmetic, where wraparound is defined
/// and yields the right answer as long as less than 2^32 ms have actually
/// passed. The naive `if (now > then)` guard that people reach for instead is
/// what breaks: 49.7 days after boot it starts reporting enormous intervals, and
/// anything gated on "has 500 ms elapsed" fires continuously.
constexpr std::uint32_t elapsed_since(Millis then, Millis now) noexcept
{
    return static_cast<std::uint32_t>(now - then);
}

/// A point in the future, expressed so that comparing against it is wrap-safe.
///
/// Deliberately not self-reading: it never calls a clock itself, it is always
/// given `now`. That keeps every user of it testable without faking time at
/// global scope, and makes the cost of reading the clock visible at the call
/// site rather than hidden inside a predicate.
class Deadline
{
public:
    Deadline() = default;

    /// Arm to expire `period_ms` after `now`.
    Deadline(Millis now, std::uint32_t period_ms) noexcept
        : start_{now}, period_ms_{period_ms}
    {
    }

    /// True once at least `period_ms` has elapsed since the last `arm()`.
    bool expired(Millis now) const noexcept
    {
        return elapsed_since(start_, now) >= period_ms_;
    }

    /// Restart the interval from `now`.
    void arm(Millis now) noexcept { start_ = now; }

    /// Restart the interval from the moment it was *due*, not from now.
    ///
    /// This is what a periodic task wants. `arm(now)` accumulates whatever
    /// lateness the scheduler introduced on every single tick, so a 10 ms task
    /// that is habitually 1 ms late silently becomes an 11 ms task and the
    /// control loop runs slower than its own constants claim.
    ///
    /// If the task has fallen behind by more than one whole period -- a long
    /// blocking call, a debugger breakpoint -- catching up would mean firing
    /// repeatedly with no delay between iterations. That is worse than the
    /// missed deadlines, so the schedule is resynchronised to `now` instead and
    /// the gap is simply skipped.
    void arm_periodic(Millis now) noexcept
    {
        const std::uint32_t late = elapsed_since(start_, now);
        if (period_ms_ == 0 || late >= 2 * period_ms_)
        {
            start_ = now;
        }
        else
        {
            start_ += period_ms_;
        }
    }

    std::uint32_t period_ms() const noexcept { return period_ms_; }
    void set_period_ms(std::uint32_t period_ms) noexcept { period_ms_ = period_ms; }

private:
    Millis start_{0};
    std::uint32_t period_ms_{0};
};

}  // namespace auk::core
