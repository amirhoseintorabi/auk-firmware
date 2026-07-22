// SPDX-License-Identifier: MIT
//
// Runs the complete AUK firmware against the simulated plant and narrates what
// happens. No hardware, no RTOS, no sleeping -- the whole scenario plays out in
// virtual time in a few milliseconds.
//
// Build and run:  cmake -B build && cmake --build build && ./build/auk-sim

#include <cstdio>
#include <string>

#include "auk/app/robot.hpp"
#include "sim/sim_board.hpp"

namespace
{

using namespace auk;

/// The pack's discharge curve. Flat through the middle, steep at both ends --
/// which is why the gauge interpolates a table instead of fitting a curve.
constexpr power::SocPoint kDischargeCurve[] = {
    {20.0F, 0.00F}, {22.5F, 0.05F}, {23.5F, 0.20F}, {24.2F, 0.50F},
    {24.9F, 0.80F}, {25.6F, 0.95F}, {26.4F, 1.00F},
};

app::RobotConfig make_config()
{
    app::RobotConfig config{};

    config.geometry.track_width_m = 0.40F;
    config.geometry.wheel_radius_m = 0.08F;
    config.geometry.max_wheel_mps = 1.20F;

    config.battery.internal_resistance_ohm = 0.02F;
    config.battery.critical_soc = 0.10F;
    config.battery.low_soc = 0.25F;
    config.battery.curve = kDischargeCurve;
    config.battery.curve_length = sizeof(kDischargeCurve) / sizeof(kDischargeCurve[0]);

    return config;
}

/// Advance both the plant and the firmware together for `ms` milliseconds.
void run_for(sim::SimBoard& sim, app::Robot& robot, std::uint32_t ms)
{
    for (std::uint32_t i = 0; i < ms; ++i)
    {
        sim.tick_ms();
        robot.step();
    }
}

const char* fault_name(safety::FaultFlags f)
{
    using safety::FaultFlags;
    if (safety::has(f, FaultFlags::SafetyQueueLost)) return "safety-queue-lost";
    if (safety::has(f, FaultFlags::EmergencyStop)) return "e-stop";
    if (safety::has(f, FaultFlags::LinkLost)) return "link-lost";
    if (safety::has(f, FaultFlags::CommandStale)) return "command-stale";
    if (safety::has(f, FaultFlags::BatteryCritical)) return "battery-critical";
    if (safety::has(f, FaultFlags::FrontBumper)) return "front-bumper";
    if (safety::has(f, FaultFlags::RearBumper)) return "rear-bumper";
    return "none";
}

void report(const char* stage, const sim::SimBoard& sim, const app::Robot& robot)
{
    const auto& pose = robot.pose();
    std::printf(
        "%-28s true(%6.3f, %6.3f, %7.3f)  odom(%6.3f, %6.3f, %7.3f)  "
        "torque=%-3s faults=%s\n",
        stage, static_cast<double>(sim.true_x_m()), static_cast<double>(sim.true_y_m()),
        static_cast<double>(sim.true_yaw_rad()), static_cast<double>(pose.x_m),
        static_cast<double>(pose.y_m), static_cast<double>(pose.yaw_rad),
        sim.torque_enabled() ? "on" : "off", fault_name(robot.faults()));
}

}  // namespace

int main()
{
    sim::SimBoard sim;
    app::Robot robot{sim.board(), make_config()};

    if (!robot.begin())
    {
        std::fprintf(stderr, "robot refused to start: incomplete board\n");
        return 1;
    }

    std::printf("AUK firmware -- simulated run\n");
    std::printf("%-28s %s\n", "", "columns: ground truth / odometry estimate");
    std::printf(
        "--------------------------------------------------"
        "--------------------------------------------------\n");

    // Nothing has been commanded yet, so the robot should be sitting still with
    // torque off. This is the state it powers up in, and it is deliberate.
    run_for(sim, robot, 100);
    report("idle at power-on", sim, robot);

    // Drive forward. Note the ramp: the demand is a step, the wheels are not.
    for (int i = 0; i < 30; ++i)
    {
        sim.send("V 0.8 0.0\n");
        run_for(sim, robot, 100);
    }
    report("after 3 s forward", sim, robot);

    // Turn in place.
    for (int i = 0; i < 20; ++i)
    {
        sim.send("V 0.0 0.8\n");
        run_for(sim, robot, 100);
    }
    report("after 2 s rotating", sim, robot);

    // Drive into something. The front bumper closes: forward motion is cut,
    // reverse and rotation stay available.
    sim.set_front_bumper(true);
    for (int i = 0; i < 10; ++i)
    {
        sim.send("V 0.8 0.0\n");
        run_for(sim, robot, 100);
    }
    report("front bumper, driving fwd", sim, robot);

    // Reverse out of it -- the same bumper, the opposite direction, permitted.
    for (int i = 0; i < 10; ++i)
    {
        sim.send("V -0.4 0.0\n");
        run_for(sim, robot, 100);
    }
    report("reversing off the bumper", sim, robot);
    sim.set_front_bumper(false);

    // Lose the link mid-command. The robot must stop on its own; nothing is
    // going to tell it to.
    for (int i = 0; i < 5; ++i)
    {
        sim.send("V 0.8 0.0\n");
        run_for(sim, robot, 100);
    }
    sim.set_link_up(false);
    run_for(sim, robot, 1000);
    report("1 s after link loss", sim, robot);

    sim.set_link_up(true);

    // E-stop, then release. Motion does not resume until the interlock is
    // satisfied *and* a fresh setpoint has arrived.
    sim.set_estop(true);
    for (int i = 0; i < 5; ++i)
    {
        sim.send("V 0.8 0.0\n");
        run_for(sim, robot, 100);
    }
    report("e-stop asserted", sim, robot);

    sim.set_estop(false);
    for (int i = 0; i < 20; ++i)
    {
        sim.send("V 0.5 0.0\n");
        run_for(sim, robot, 100);
    }
    report("2 s after e-stop release", sim, robot);

    // Flatten the pack. Below the critical threshold, motion stops.
    sim.set_battery_volts(21.0F);
    for (int i = 0; i < 10; ++i)
    {
        sim.send("V 0.5 0.0\n");
        run_for(sim, robot, 100);
    }
    report("battery critical", sim, robot);

    std::printf(
        "--------------------------------------------------"
        "--------------------------------------------------\n");
    std::printf("watchdog resets: %u (0 means the loop never missed a deadline)\n",
                sim.watchdog_resets());

    const std::string tail = sim.take_output();
    const std::size_t nl = tail.find_last_of('\n', tail.empty() ? 0 : tail.size() - 2);
    if (nl != std::string::npos && nl + 1 < tail.size())
    {
        std::printf("last telemetry frame:\n  %s", tail.c_str() + nl + 1);
    }

    return 0;
}
