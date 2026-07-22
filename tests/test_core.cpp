// SPDX-License-Identifier: MIT
//
// Ring buffer, command bus and deadline arithmetic.

#include "tests/check.hpp"

#include "auk/core/command.hpp"
#include "auk/core/deadline.hpp"
#include "auk/core/ring_buffer.hpp"

using namespace auk;
using namespace auk::core;

namespace
{

void test_ring_buffer_basics()
{
    CASE("ring buffer returns elements in order");
    RingBuffer<int, 4> rb;
    CHECK(rb.empty());
    CHECK(rb.size() == 0);

    for (int i = 1; i <= 4; ++i)
    {
        CHECK(ok(rb.push(i)));
    }
    CHECK(rb.full());
    CHECK(rb.size() == 4);

    for (int expected = 1; expected <= 4; ++expected)
    {
        int value = 0;
        CHECK(ok(rb.pop(value)));
        CHECK(value == expected);
    }
    CHECK(rb.empty());
}

void test_ring_buffer_reject()
{
    CASE("Reject policy refuses a push into a full buffer");
    RingBuffer<int, 2, OverflowPolicy::Reject> rb;
    CHECK(ok(rb.push(1)));
    CHECK(ok(rb.push(2)));
    CHECK(rb.push(3) == Result::Full);
    CHECK(rb.size() == 2);
    CHECK(rb.dropped() == 0);

    // The rejected element must not have displaced anything.
    int value = 0;
    CHECK(ok(rb.pop(value)));
    CHECK(value == 1);
}

void test_ring_buffer_drop_oldest()
{
    CASE("DropOldest keeps the newest elements and counts the losses");
    RingBuffer<int, 3, OverflowPolicy::DropOldest> rb;
    for (int i = 1; i <= 6; ++i)
    {
        CHECK(ok(rb.push(i)));
    }
    CHECK(rb.size() == 3);
    CHECK(rb.dropped() == 3);

    for (int expected = 4; expected <= 6; ++expected)
    {
        int value = 0;
        CHECK(ok(rb.pop(value)));
        CHECK(value == expected);
    }
}

void test_ring_buffer_capacity_one()
{
    CASE("a one-slot mailbox holds only the newest value");
    RingBuffer<int, 1, OverflowPolicy::DropOldest> rb;
    for (int i = 1; i <= 5; ++i)
    {
        CHECK(ok(rb.push(i)));
        CHECK(rb.size() == 1);
    }

    int value = 0;
    CHECK(ok(rb.pop(value)));
    CHECK(value == 5);
    CHECK(rb.pop(value) == Result::Empty);
}

void test_ring_buffer_drain_refill()
{
    CASE("indices stay consistent across repeated drain and refill");
    // The cycle length is coprime with the capacity, so head and tail land on
    // every alignment rather than repeating the same one.
    RingBuffer<int, 4> rb;
    int stamp = 0;
    for (int cycle = 0; cycle < 20; ++cycle)
    {
        const int n = (cycle % 4) + 1;
        for (int i = 0; i < n; ++i)
        {
            CHECK(ok(rb.push(++stamp)));
        }
        for (int i = 0; i < n; ++i)
        {
            int value = 0;
            CHECK(ok(rb.pop(value)));
            CHECK(value == stamp - n + 1 + i);
        }
        CHECK(rb.empty());
    }
}

void test_ring_buffer_peek()
{
    CASE("peek does not consume");
    RingBuffer<int, 2> rb;
    int value = 0;
    CHECK(rb.peek(value) == Result::Empty);

    CHECK(ok(rb.push(7)));
    CHECK(ok(rb.peek(value)));
    CHECK(value == 7);
    CHECK(rb.size() == 1);
    CHECK(ok(rb.peek(value)));
    CHECK(rb.size() == 1);
}

void test_deadline_expiry()
{
    CASE("a deadline expires once its period has elapsed");
    Deadline d{1000, 50};
    CHECK(!d.expired(1000));
    CHECK(!d.expired(1049));
    CHECK(d.expired(1050));
    CHECK(d.expired(5000));

    d.arm(5000);
    CHECK(!d.expired(5049));
    CHECK(d.expired(5050));
}

void test_deadline_wraparound()
{
    CASE("deadlines survive the 32-bit millisecond wrap");
    // Armed 10 ms before the counter wraps; must expire 50 ms later, which is
    // 40 ms *after* the wrap. A naive now > then comparison fails right here.
    constexpr Millis near_wrap = 0xFFFFFFF6U;  // UINT32_MAX - 9
    Deadline d{near_wrap, 50};

    CHECK(!d.expired(near_wrap));
    CHECK(!d.expired(5));  // 15 ms elapsed, having wrapped
    CHECK(d.expired(40));  // 50 ms elapsed
    CHECK(elapsed_since(near_wrap, 40) == 50);
}

void test_deadline_periodic_no_drift()
{
    CASE("arm_periodic does not accumulate scheduler lateness");
    Deadline d{0, 10};

    // Fire consistently 3 ms late. The schedule must stay on the 10 ms grid
    // rather than drifting out to 13 ms per iteration.
    Millis now = 13;
    for (int i = 0; i < 5; ++i)
    {
        CHECK(d.expired(now));
        d.arm_periodic(now);
        now += 13;
    }
    // After 5 fires the schedule should be at 50 ms, not 65 ms, so it is already
    // due at 50.
    CHECK(d.expired(50));
}

void test_deadline_periodic_resync()
{
    CASE("arm_periodic resynchronises after a long stall");
    Deadline d{0, 10};

    // Stalled for 500 ms. Catching up would mean firing 50 times back to back;
    // instead the schedule restarts from now.
    d.arm_periodic(500);
    CHECK(!d.expired(505));
    CHECK(d.expired(510));
}

void test_command_bus_priority()
{
    CASE("each lane is drained independently");
    CommandBus bus;

    Command high{};
    high.type = CommandType::SetEmergencyStop;
    high.priority = Priority::High;
    CHECK(ok(bus.post(high)));

    Command normal{};
    normal.type = CommandType::SetVelocity;
    normal.priority = Priority::Normal;
    CHECK(ok(bus.post(normal)));

    Command out{};
    CHECK(ok(bus.take(Priority::High, out)));
    CHECK(out.type == CommandType::SetEmergencyStop);
    CHECK(bus.take(Priority::High, out) == Result::Empty);

    CHECK(ok(bus.take(Priority::Normal, out)));
    CHECK(out.type == CommandType::SetVelocity);
}

void test_command_bus_normal_lane_keeps_newest()
{
    CASE("the velocity lane keeps only the newest setpoint");
    CommandBus bus;

    for (int i = 1; i <= 5; ++i)
    {
        Command c{};
        c.type = CommandType::SetVelocity;
        c.priority = Priority::Normal;
        c.values[0] = static_cast<float>(i);
        CHECK(ok(bus.post(c)));
    }

    Command out{};
    CHECK(ok(bus.take(Priority::Normal, out)));
    CHECK_NEAR(out.values[0], 5.0, 1e-6);
    CHECK(bus.take(Priority::Normal, out) == Result::Empty);
    CHECK(bus.dropped() == 4);
}

void test_command_bus_safety_lane_never_overwrites()
{
    CASE("the safety lane rejects rather than discarding an event");
    CommandBus bus;

    // Fill it, then overflow it.
    int accepted = 0;
    for (int i = 0; i < 64; ++i)
    {
        Command c{};
        c.type = CommandType::BumperChanged;
        c.priority = Priority::High;
        c.values[0] = static_cast<float>(i);
        if (ok(bus.post(c)))
        {
            ++accepted;
        }
    }

    CHECK(accepted > 0);
    CHECK(bus.safety_lane_overflowed());

    // The elements that were accepted must be the *earliest* ones, unchanged:
    // rejecting means the queue's existing contents were never disturbed.
    Command out{};
    CHECK(ok(bus.take(Priority::High, out)));
    CHECK_NEAR(out.values[0], 0.0, 1e-6);
}

}  // namespace

int main()
{
    std::printf("core\n");
    test_ring_buffer_basics();
    test_ring_buffer_reject();
    test_ring_buffer_drop_oldest();
    test_ring_buffer_capacity_one();
    test_ring_buffer_drain_refill();
    test_ring_buffer_peek();
    test_deadline_expiry();
    test_deadline_wraparound();
    test_deadline_periodic_no_drift();
    test_deadline_periodic_resync();
    test_command_bus_priority();
    test_command_bus_normal_lane_keeps_newest();
    test_command_bus_safety_lane_never_overwrites();
    return auk::test::summary("core");
}
