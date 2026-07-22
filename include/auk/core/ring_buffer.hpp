// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>

#include "auk/core/result.hpp"

namespace auk::core
{

/// What a ring buffer does when a push arrives and it is already full.
enum class OverflowPolicy : std::uint8_t
{
    /// Reject the new element and report `Result::Full`. Correct when every
    /// element matters -- fault codes, audit events.
    Reject,

    /// Drop the oldest element to make room. Correct when only recency matters
    /// -- a velocity setpoint two cycles stale is worth less than the one that
    /// just arrived.
    DropOldest,
};

/// Fixed-capacity single-producer / single-consumer ring buffer.
///
/// Storage is a member array, so there is no allocation and the footprint is
/// known at link time. `Capacity` is the number of elements, not a power-of-two
/// mask, so `Capacity == 1` is legal and useful (a mailbox holding only the
/// newest value).
///
/// Concurrency: safe for exactly one producer and one consumer, which may be an
/// ISR and a thread respectively, **provided** `T` is trivially copyable and the
/// platform's word-sized loads and stores are atomic. It is not safe for two
/// producers. `CommandBus` documents which side is which for its own use.
///
/// The count is tracked explicitly rather than being derived from the two
/// indices. The derived form needs either a sacrificial slot or a wrap-parity
/// bit to tell "full" from "empty", and getting that subtly wrong is a classic
/// way to end up with a buffer that silently reports itself empty while holding
/// a full set of unread elements.
template <typename T, std::size_t Capacity,
          OverflowPolicy Policy = OverflowPolicy::Reject>
class RingBuffer
{
    static_assert(Capacity > 0, "a ring buffer needs at least one slot");

public:
    using value_type = T;

    static constexpr std::size_t capacity() noexcept { return Capacity; }

    std::size_t size() const noexcept { return count_; }
    bool empty() const noexcept { return count_ == 0; }
    bool full() const noexcept { return count_ == Capacity; }

    /// Number of elements discarded by `OverflowPolicy::DropOldest` since
    /// construction. Saturates rather than wrapping, so a monitoring task can
    /// treat any non-zero value as "this buffer is undersized for its load"
    /// without worrying about a counter that rolled over back to zero.
    std::uint32_t dropped() const noexcept { return dropped_; }

    /// Append an element.
    ///
    /// Returns `Result::Ok` on success, or `Result::Full` under
    /// `OverflowPolicy::Reject` when there was no room. Under
    /// `OverflowPolicy::DropOldest` this always returns `Result::Ok`; check
    /// `dropped()` to find out whether anything was lost.
    Result push(const T& value) noexcept
    {
        if (count_ == Capacity)
        {
            if constexpr (Policy == OverflowPolicy::Reject)
            {
                return Result::Full;
            }
            else
            {
                // Retire the oldest element. head_ advances, count_ stays at
                // capacity, and the two indices stay consistent with each other
                // -- which is the property that matters, because an index pair
                // that drifts apart stays wrong forever.
                head_ = next(head_);
                --count_;
                if (dropped_ != UINT32_MAX)
                {
                    ++dropped_;
                }
            }
        }

        slots_[tail_] = value;
        tail_ = next(tail_);
        ++count_;
        return Result::Ok;
    }

    /// Copy out the oldest element and remove it.
    /// Returns `Result::Empty` if there is nothing to read; `out` is untouched.
    Result pop(T& out) noexcept
    {
        if (count_ == 0)
        {
            return Result::Empty;
        }

        out = slots_[head_];
        head_ = next(head_);
        --count_;
        return Result::Ok;
    }

    /// Copy out the oldest element without removing it.
    Result peek(T& out) const noexcept
    {
        if (count_ == 0)
        {
            return Result::Empty;
        }

        out = slots_[head_];
        return Result::Ok;
    }

    /// Discard everything. Does not reset `dropped()`, which is a lifetime
    /// statistic rather than a queue state.
    void clear() noexcept
    {
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

private:
    static constexpr std::size_t next(std::size_t index) noexcept
    {
        // Capacity is rarely a power of two here, and a modulo by a runtime
        // value would be a division on Cortex-M0. A compare-and-wrap is one
        // branch and works for any capacity.
        return (index + 1 == Capacity) ? 0 : index + 1;
    }

    T slots_[Capacity]{};
    std::size_t head_{0};  ///< index of the oldest element
    std::size_t tail_{0};  ///< index the next push will write to
    std::size_t count_{0};
    std::uint32_t dropped_{0};
};

}  // namespace auk::core
