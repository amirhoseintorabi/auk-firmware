// SPDX-License-Identifier: MIT
//
// The interlock truth table, and the battery gauge.
//
// This is the file to read first if you want to know what the robot will and
// will not do. Every row here is a claim about behaviour that somebody could
// otherwise only discover by standing in front of the machine.

#include "tests/check.hpp"

#include "auk/power/battery_gauge.hpp"
#include "auk/safety/interlock.hpp"

using namespace auk;
using namespace auk::safety;

namespace
{

/// Inputs describing a healthy robot that has just been commanded.
SafetyInputs healthy(core::Millis now)
{
    SafetyInputs in{};
    in.estop_asserted = false;
    in.front_bumper_pressed = false;
    in.rear_bumper_pressed = false;
    in.link_up = true;
    in.battery_critical = false;
    in.safety_queue_overflowed = false;
    in.last_command_ms = now;
    return in;
}

void test_starts_faulted()
{
    CASE("the interlock is faulted before it has seen any input");
    // Power-on state. A robot that permits motion because nothing has told it
    // not to is a robot that moves during a brownout.
    Interlock lock;
    CHECK(!lock.motion_allowed());
    CHECK(any(lock.faults()));

    const control::Twist out = lock.permit({1.0F, 1.0F});
    CHECK_NEAR(out.linear_mps, 0.0, 1e-9);
    CHECK_NEAR(out.angular_rps, 0.0, 1e-9);
}

void test_healthy_permits_motion()
{
    CASE("a healthy robot passes the requested twist through unchanged");
    Interlock lock;
    lock.update(healthy(1000), 1000);

    CHECK(lock.motion_allowed());
    CHECK(lock.faults() == FaultFlags::None);

    const control::Twist out = lock.permit({0.5F, -0.3F});
    CHECK_NEAR(out.linear_mps, 0.5, 1e-6);
    CHECK_NEAR(out.angular_rps, -0.3, 1e-6);
}

void test_estop_stops_everything()
{
    CASE("e-stop zeroes both axes");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.estop_asserted = true;
    lock.update(in, 1000);

    CHECK(!lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::EmergencyStop));

    const control::Twist out = lock.permit({0.5F, 0.5F});
    CHECK_NEAR(out.linear_mps, 0.0, 1e-9);
    CHECK_NEAR(out.angular_rps, 0.0, 1e-9);
}

void test_front_bumper_blocks_forward_only()
{
    CASE("a front bumper blocks forward motion and permits escape");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.front_bumper_pressed = true;
    lock.update(in, 1000);

    // A bumper is not a blocking fault -- the robot is still allowed to move,
    // just not into the thing it hit.
    CHECK(lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::FrontBumper));

    const control::Twist forward = lock.permit({0.5F, 0.0F});
    CHECK_NEAR(forward.linear_mps, 0.0, 1e-9);

    const control::Twist reverse = lock.permit({-0.5F, 0.0F});
    CHECK_NEAR(reverse.linear_mps, -0.5, 1e-6);

    // Rotation stays available so the operator can point the robot away.
    const control::Twist turning = lock.permit({0.0F, 0.7F});
    CHECK_NEAR(turning.angular_rps, 0.7, 1e-6);
}

void test_rear_bumper_blocks_reverse_only()
{
    CASE("a rear bumper is the mirror image");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.rear_bumper_pressed = true;
    lock.update(in, 1000);

    CHECK_NEAR(lock.permit({-0.5F, 0.0F}).linear_mps, 0.0, 1e-9);
    CHECK_NEAR(lock.permit({0.5F, 0.0F}).linear_mps, 0.5, 1e-6);
}

void test_both_bumpers_block_both_directions()
{
    CASE("both bumpers leave only rotation");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.front_bumper_pressed = true;
    in.rear_bumper_pressed = true;
    lock.update(in, 1000);

    CHECK_NEAR(lock.permit({0.5F, 0.4F}).linear_mps, 0.0, 1e-9);
    CHECK_NEAR(lock.permit({-0.5F, 0.4F}).linear_mps, 0.0, 1e-9);
    CHECK_NEAR(lock.permit({0.0F, 0.4F}).angular_rps, 0.4, 1e-6);
}

void test_link_loss_stops_the_robot()
{
    CASE("losing the link stops the robot");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.link_up = false;
    lock.update(in, 1000);

    CHECK(!lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::LinkLost));
}

void test_stale_commands_stop_the_robot()
{
    CASE("a live link with no fresh setpoints still stops the robot");
    // The case a link-up check alone misses entirely: the transport is fine and
    // the controller behind it has died. Without this the robot keeps driving on
    // the last thing it was told, indefinitely.
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.link_up = true;

    lock.update(in, 1000 + Interlock::kCommandTimeoutMs);
    CHECK(lock.motion_allowed());

    lock.update(in, 1000 + Interlock::kCommandTimeoutMs + 1);
    CHECK(!lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::CommandStale));
}

void test_command_timeout_survives_wraparound()
{
    CASE("the command timeout is correct across the millisecond wrap");
    Interlock lock;
    constexpr core::Millis near_wrap = 0xFFFFFF00U;

    SafetyInputs in = healthy(near_wrap);
    in.last_command_ms = near_wrap;

    // 200 ms later, having wrapped past zero: still inside the timeout.
    lock.update(in, near_wrap + 200);
    CHECK(!has(lock.faults(), FaultFlags::CommandStale));

    // 400 ms later: outside it.
    lock.update(in, near_wrap + 400);
    CHECK(has(lock.faults(), FaultFlags::CommandStale));
}

void test_critical_battery_stops_the_robot()
{
    CASE("a critical pack stops the robot");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.battery_critical = true;
    lock.update(in, 1000);

    CHECK(!lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::BatteryCritical));
}

void test_lost_safety_event_stops_the_robot()
{
    CASE("a dropped safety event stops the robot");
    // If the safety lane overflowed we cannot know what was lost, so the only
    // defensible response is to stop and require human intervention.
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.safety_queue_overflowed = true;
    lock.update(in, 1000);

    CHECK(!lock.motion_allowed());
    CHECK(has(lock.faults(), FaultFlags::SafetyQueueLost));
}

void test_all_active_faults_are_reported()
{
    CASE("every active fault is reported, not just the first");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.estop_asserted = true;
    in.front_bumper_pressed = true;
    in.link_up = false;
    in.battery_critical = true;
    lock.update(in, 1000);

    const FaultFlags f = lock.faults();
    CHECK(has(f, FaultFlags::EmergencyStop));
    CHECK(has(f, FaultFlags::FrontBumper));
    CHECK(has(f, FaultFlags::LinkLost));
    CHECK(has(f, FaultFlags::BatteryCritical));
}

void test_faults_clear_when_conditions_clear()
{
    CASE("faults clear once their cause goes away");
    Interlock lock;
    SafetyInputs in = healthy(1000);
    in.estop_asserted = true;
    lock.update(in, 1000);
    CHECK(!lock.motion_allowed());

    in.estop_asserted = false;
    in.last_command_ms = 2000;
    lock.update(in, 2000);
    CHECK(lock.motion_allowed());
    CHECK(lock.faults() == FaultFlags::None);
}

// ---------------------------------------------------------------- battery ---

constexpr power::SocPoint kCurve[] = {
    {20.0F, 0.00F}, {22.5F, 0.05F}, {23.5F, 0.20F}, {24.2F, 0.50F},
    {24.9F, 0.80F}, {25.6F, 0.95F}, {26.4F, 1.00F},
};

power::BatteryConfig battery_config()
{
    power::BatteryConfig c{};
    c.internal_resistance_ohm = 0.02F;
    c.critical_soc = 0.10F;
    c.low_soc = 0.25F;
    c.curve = kCurve;
    c.curve_length = sizeof(kCurve) / sizeof(kCurve[0]);
    return c;
}

void test_gauge_compensates_for_load()
{
    CASE("the gauge removes the IR drop before reading the curve");
    // Same pack, two loads. Without compensation the loaded reading would look
    // like a materially emptier battery.
    power::BatteryGauge idle{battery_config()};
    idle.update(24.9F, 0.0F);

    power::BatteryGauge loaded{battery_config()};
    loaded.update(24.9F - (20.0F * 0.02F), 20.0F);  // 0.4 V of sag at 20 A

    CHECK_NEAR(idle.state_of_charge(), loaded.state_of_charge(), 1e-4);
}

void test_gauge_is_monotonic_under_noise()
{
    CASE("the reading does not creep upward on noise");
    power::BatteryGauge gauge{battery_config()};
    gauge.update(24.2F, 0.0F);
    const float first = gauge.state_of_charge();

    // A small rise, repeatedly. Must be ignored.
    for (int i = 0; i < 50; ++i)
    {
        gauge.update(24.25F, 0.0F);
    }
    CHECK(gauge.state_of_charge() <= first + 1e-6F);
}

void test_gauge_accepts_a_pack_change()
{
    CASE("a large sustained rise is accepted as a new pack");
    power::BatteryGauge gauge{battery_config()};
    gauge.update(23.5F, 0.0F);
    CHECK(gauge.state_of_charge() < 0.5F);

    // Somebody swapped in a full pack. One sample is not enough; a sustained
    // run of them is.
    for (int i = 0; i < 20; ++i)
    {
        gauge.update(26.4F, 0.0F);
    }
    CHECK(gauge.state_of_charge() > 0.9F);
}

void test_gauge_thresholds()
{
    CASE("low and critical thresholds behave");
    power::BatteryGauge gauge{battery_config()};

    gauge.update(25.6F, 0.0F);
    CHECK(!gauge.is_low());
    CHECK(!gauge.is_critical());

    gauge.update(23.5F, 0.0F);
    CHECK(gauge.is_low());
    CHECK(!gauge.is_critical());

    gauge.update(21.0F, 0.0F);
    CHECK(gauge.is_critical());
}

void test_gauge_clamps_outside_the_curve()
{
    CASE("voltages beyond the characterised range clamp rather than extrapolate");
    power::BatteryGauge high{battery_config()};
    high.update(30.0F, 0.0F);
    CHECK_NEAR(high.state_of_charge(), 1.0, 1e-6);

    power::BatteryGauge low{battery_config()};
    low.update(10.0F, 0.0F);
    CHECK_NEAR(low.state_of_charge(), 0.0, 1e-6);
}

void test_gauge_reports_no_reading_before_first_sample()
{
    CASE("the gauge admits when it has not measured anything yet");
    power::BatteryGauge gauge{battery_config()};
    CHECK(!gauge.has_reading());
    gauge.update(24.2F, 0.0F);
    CHECK(gauge.has_reading());
}

}  // namespace

int main()
{
    std::printf("safety and power\n");
    test_starts_faulted();
    test_healthy_permits_motion();
    test_estop_stops_everything();
    test_front_bumper_blocks_forward_only();
    test_rear_bumper_blocks_reverse_only();
    test_both_bumpers_block_both_directions();
    test_link_loss_stops_the_robot();
    test_stale_commands_stop_the_robot();
    test_command_timeout_survives_wraparound();
    test_critical_battery_stops_the_robot();
    test_lost_safety_event_stops_the_robot();
    test_all_active_faults_are_reported();
    test_faults_clear_when_conditions_clear();
    test_gauge_compensates_for_load();
    test_gauge_is_monotonic_under_noise();
    test_gauge_accepts_a_pack_change();
    test_gauge_thresholds();
    test_gauge_clamps_outside_the_curve();
    test_gauge_reports_no_reading_before_first_sample();
    return auk::test::summary("safety and power");
}
