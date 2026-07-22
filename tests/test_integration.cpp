// SPDX-License-Identifier: MIT
//
// End-to-end tests: the real firmware driving the simulated plant.
//
// These are the ones that catch wiring mistakes -- a module that is correct in
// isolation but never called, or called in the wrong order. Every scenario here
// is one that would otherwise need a robot, a clear floor and a willing
// volunteer.

#include "tests/check.hpp"

#include <string>

#include "auk/app/robot.hpp"
#include "sim/sim_board.hpp"

using namespace auk;

namespace
{

constexpr power::SocPoint kCurve[] = {
    {20.0F, 0.00F}, {22.5F, 0.05F}, {23.5F, 0.20F}, {24.2F, 0.50F},
    {24.9F, 0.80F}, {25.6F, 0.95F}, {26.4F, 1.00F},
};

app::RobotConfig config()
{
    app::RobotConfig c{};
    c.geometry.track_width_m = 0.40F;
    c.geometry.wheel_radius_m = 0.08F;
    c.geometry.max_wheel_mps = 1.20F;
    c.battery.internal_resistance_ohm = 0.02F;
    c.battery.critical_soc = 0.10F;
    c.battery.low_soc = 0.25F;
    c.battery.curve = kCurve;
    c.battery.curve_length = sizeof(kCurve) / sizeof(kCurve[0]);
    return c;
}

/// Advance plant and firmware together.
void run(sim::SimBoard& s, app::Robot& r, std::uint32_t ms)
{
    for (std::uint32_t i = 0; i < ms; ++i)
    {
        s.tick_ms();
        r.step();
    }
}

/// Hold a setpoint for `ms`, refreshing it often enough not to go stale.
void drive(sim::SimBoard& s, app::Robot& r, const char* command, std::uint32_t ms)
{
    const std::uint32_t slice = 50;
    for (std::uint32_t elapsed = 0; elapsed < ms; elapsed += slice)
    {
        s.send(command);
        run(s, r, slice);
    }
}

void test_refuses_to_start_on_incomplete_board()
{
    CASE("an incomplete board refuses to start rather than running blind");
    hal::Board empty{};
    app::Robot robot{empty, config()};
    CHECK(!robot.begin());
    CHECK(!robot.step());
}

void test_powers_up_stopped()
{
    CASE("the robot powers up with torque off and stays put");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    run(s, r, 500);

    CHECK(!s.torque_enabled());
    CHECK_NEAR(s.true_x_m(), 0.0, 1e-6);
    CHECK(!safety::has(r.faults(), safety::FaultFlags::EmergencyStop));
    // Nothing has commanded it, so the stale-command fault is what holds it.
    CHECK(safety::has(r.faults(), safety::FaultFlags::CommandStale));
}

void test_drives_forward_on_command()
{
    CASE("a velocity command moves the robot forward");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 3000);

    CHECK(s.torque_enabled());
    CHECK(s.true_x_m() > 1.0F);
    CHECK_NEAR(s.true_y_m(), 0.0, 0.05);

    // Odometry should agree with ground truth to within the wheel lag.
    CHECK_NEAR(r.pose().x_m, s.true_x_m(), 0.05);
}

void test_odometry_tracks_a_turn()
{
    CASE("odometry tracks a turn against ground truth");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.4 0.5\n", 4000);

    CHECK_NEAR(r.pose().x_m, s.true_x_m(), 0.10);
    CHECK_NEAR(r.pose().y_m, s.true_y_m(), 0.10);
    CHECK_NEAR(r.pose().yaw_rad, s.true_yaw_rad(), 0.05);
}

void test_stops_when_commands_stop()
{
    CASE("the robot stops on its own when setpoints stop arriving");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 1000);
    CHECK(s.torque_enabled());

    const float x_when_abandoned = s.true_x_m();
    run(s, r, 2000);  // nothing sent

    CHECK(!s.torque_enabled());
    CHECK(safety::has(r.faults(), safety::FaultFlags::CommandStale));

    // It coasted a little and then stayed put.
    const float x_after = s.true_x_m();
    run(s, r, 2000);
    CHECK_NEAR(s.true_x_m(), x_after, 1e-3);
    CHECK(x_after > x_when_abandoned);
}

void test_stops_on_link_loss()
{
    CASE("the robot stops when the link drops mid-command");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 1000);
    CHECK(s.torque_enabled());

    s.set_link_up(false);
    run(s, r, 500);

    CHECK(!s.torque_enabled());
    CHECK(safety::has(r.faults(), safety::FaultFlags::LinkLost));
}

void test_bumper_blocks_forward_but_allows_escape()
{
    CASE("a front bumper stops forward motion and still allows reversing out");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 1000);
    const float x_at_contact = s.true_x_m();

    s.set_front_bumper(true);
    drive(s, r, "V 0.6 0.0\n", 1500);

    // It must stop, but it cannot stop instantly, and the bound here is derived
    // rather than guessed:
    //
    //   ramp    v^2 / 2a = 0.36 / (2 * 2.0)          = 0.090 m
    //   lag     v * tau  = 0.6 * 0.08                = 0.048 m
    //                                          total ~ 0.138 m
    //
    // 0.20 m leaves room for the discrete time step without being so loose that
    // a genuine regression -- the interlock not firing at all, which would give
    // metres -- could hide under it. If this ever fails, check the decel limit
    // and the simulated wheel time constant before touching the number.
    const float coast = s.true_x_m() - x_at_contact;
    CHECK(coast < 0.20F);
    CHECK(coast > 0.0F);  // it really was moving when the bumper closed
    CHECK(safety::has(r.faults(), safety::FaultFlags::FrontBumper));

    // Reversing is still permitted, which is how the operator recovers.
    const float x_before_reverse = s.true_x_m();
    drive(s, r, "V -0.4 0.0\n", 1500);
    CHECK(s.true_x_m() < x_before_reverse - 0.10F);
}

void test_estop_stops_and_requires_release()
{
    CASE("e-stop cuts torque and motion does not resume by itself");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 1000);
    CHECK(s.torque_enabled());

    s.set_estop(true);
    drive(s, r, "V 0.6 0.0\n", 500);
    CHECK(!s.torque_enabled());

    // Commands keep arriving throughout; they must not re-arm the drivetrain
    // while the e-stop is held.
    drive(s, r, "V 0.6 0.0\n", 1000);
    CHECK(!s.torque_enabled());

    s.set_estop(false);
    drive(s, r, "V 0.6 0.0\n", 500);
    CHECK(s.torque_enabled());
}

void test_software_estop_over_the_link()
{
    CASE("an e-stop arriving over the link is honoured, and C clears it");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.6 0.0\n", 1000);
    CHECK(s.torque_enabled());

    s.send("E 1\n");
    drive(s, r, "V 0.6 0.0\n", 500);
    CHECK(!s.torque_enabled());
    CHECK(safety::has(r.faults(), safety::FaultFlags::EmergencyStop));

    s.send("C\n");
    drive(s, r, "V 0.6 0.0\n", 500);
    CHECK(s.torque_enabled());
}

void test_critical_battery_stops_the_robot()
{
    CASE("a critical pack stops the robot");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.5 0.0\n", 1000);
    CHECK(s.torque_enabled());

    s.set_battery_volts(21.0F);
    drive(s, r, "V 0.5 0.0\n", 1000);

    CHECK(!s.torque_enabled());
    CHECK(safety::has(r.faults(), safety::FaultFlags::BatteryCritical));
}

void test_ramp_limits_the_step()
{
    CASE("a step command is ramped rather than slammed through");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    // One control period after a full-speed request, the commanded twist must
    // still be near zero: 0.8 m/s^2 for 10 ms is 8 mm/s.
    s.send("V 1.2 0.0\n");
    run(s, r, 30);

    CHECK(r.last_telemetry().commanded.linear_mps < 0.10F);

    drive(s, r, "V 1.2 0.0\n", 3000);
    CHECK(r.last_telemetry().commanded.linear_mps > 0.9F);
}

void test_lamp_is_controllable()
{
    CASE("the lamp follows its command");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    CHECK(!s.lamp_on());
    s.send("L 1\n");
    run(s, r, 100);
    CHECK(s.lamp_on());

    s.send("L 0\n");
    run(s, r, 100);
    CHECK(!s.lamp_on());
}

void test_watchdog_is_kicked()
{
    CASE("a healthy loop keeps the watchdog satisfied");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.4 0.0\n", 5000);
    CHECK(s.watchdog_resets() == 0);
}

void test_watchdog_fires_when_the_loop_stops()
{
    CASE("a loop that stops running trips the watchdog");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    // Advance the plant without ever calling step() -- the firmware equivalent
    // of a wedged control loop.
    s.advance(1000);
    CHECK(s.watchdog_resets() > 0);
}

void test_telemetry_is_emitted_and_parseable()
{
    CASE("telemetry frames are emitted and carry a rising sequence number");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    drive(s, r, "V 0.4 0.0\n", 1000);

    const std::string out = s.take_output();
    CHECK(!out.empty());
    CHECK(out.find("S ") != std::string::npos);
    CHECK(out.find("x=") != std::string::npos);
    CHECK(out.find("faults=") != std::string::npos);
    CHECK(r.last_telemetry().sequence > 1);
}

void test_malformed_input_is_ignored()
{
    CASE("malformed and hostile input moves nothing");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    s.send("V nan 0.0\n");
    s.send("V inf inf\n");
    s.send("V 1.0\n");         // missing an argument
    s.send("V 1.0 2.0abc\n");  // trailing rubbish
    s.send("VELOCITY 1 1\n");  // not the protocol
    s.send("\n\n\n");
    run(s, r, 500);

    CHECK(!s.torque_enabled());
    CHECK_NEAR(s.true_x_m(), 0.0, 1e-6);
}

void test_overlong_line_is_discarded_whole()
{
    CASE("an over-length line is discarded rather than truncated into a command");
    sim::SimBoard s;
    app::Robot r{s.board(), config()};
    CHECK(r.begin());

    // A truncating parser would cut this into something valid and quite
    // different. It must move nothing.
    std::string flood = "V 0.9 0.0";
    flood.append(400, '0');
    flood.push_back('\n');
    s.send(flood);
    run(s, r, 500);

    CHECK(!s.torque_enabled());
    CHECK_NEAR(s.true_x_m(), 0.0, 1e-6);
}

}  // namespace

int main()
{
    std::printf("integration\n");
    test_refuses_to_start_on_incomplete_board();
    test_powers_up_stopped();
    test_drives_forward_on_command();
    test_odometry_tracks_a_turn();
    test_stops_when_commands_stop();
    test_stops_on_link_loss();
    test_bumper_blocks_forward_but_allows_escape();
    test_estop_stops_and_requires_release();
    test_software_estop_over_the_link();
    test_critical_battery_stops_the_robot();
    test_ramp_limits_the_step();
    test_lamp_is_controllable();
    test_watchdog_is_kicked();
    test_watchdog_fires_when_the_loop_stops();
    test_telemetry_is_emitted_and_parseable();
    test_malformed_input_is_ignored();
    test_overlong_line_is_discarded_whole();
    return auk::test::summary("integration");
}
