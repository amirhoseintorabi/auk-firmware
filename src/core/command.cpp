// SPDX-License-Identifier: MIT
#include "auk/core/command.hpp"

namespace auk::core
{

Result CommandBus::post(const Command& command) noexcept
{
    switch (command.priority)
    {
        case Priority::High:
        {
            const Result r = high_.push(command);
            if (r == Result::Full)
            {
                // Latched, never cleared. If the safety lane has overflowed even
                // once then a discrete event -- an e-stop press, a bumper strike
                // -- was thrown away, and no later success makes that untrue.
                high_overflowed_ = true;
            }
            return r;
        }

        case Priority::Normal:
            return normal_.push(command);

        case Priority::Low:
            return low_.push(command);

        case Priority::Count:
            break;
    }

    return Result::OutOfRange;
}

Result CommandBus::take(Priority lane, Command& out) noexcept
{
    switch (lane)
    {
        case Priority::High:
            return high_.pop(out);
        case Priority::Normal:
            return normal_.pop(out);
        case Priority::Low:
            return low_.pop(out);
        case Priority::Count:
            break;
    }

    return Result::OutOfRange;
}

std::uint32_t CommandBus::dropped() const noexcept
{
    // Saturating, matching RingBuffer::dropped(): a diagnostic counter that
    // wraps to zero is worse than one that sticks at the maximum, because zero
    // reads as "healthy".
    const std::uint64_t total =
        static_cast<std::uint64_t>(normal_.dropped()) + low_.dropped();

    return (total > UINT32_MAX) ? UINT32_MAX : static_cast<std::uint32_t>(total);
}

}  // namespace auk::core
