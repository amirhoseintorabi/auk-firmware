// SPDX-License-Identifier: MIT
//
// Kinematics, odometry and slew limiting.

#include "tests/check.hpp"

#include <cmath>

#include "auk/control/differential_drive.hpp"
#include "auk/control/odometry.hpp"
#include "auk/control/slew_limiter.hpp"

using namespace auk;
using namespace auk::control;

namespace
{

constexpr float kPi = 3.14159265358979323846F;

DriveGeometry geometry()
{
    DriveGeometry g{};
    g.track_width_m = 0.40F;
    g.wheel_radius_m = 0.08F;
    g.max_wheel_mps = 1.20F;
    return g;
}

void test_straight_line_has_no_differential()
{
    CASE("pure forward motion drives both wheels equally");
    DifferentialDrive drive{geometry()};
    const WheelSpeeds w = drive.to_wheels({0.5F, 0.0F});
    CHECK_NEAR(w.left_mps, 0.5, 1e-6);
    CHECK_NEAR(w.right_mps, 0.5, 1e-6);
}

void test_spin_uses_half_track()
{
    CASE("yaw uses the half track as its moment arm");
    // The check that catches the classic factor-of-two error. With a 0.40 m
    // track, 1 rad/s on the spot must give +/- 0.20 m/s at the wheels -- not
    // +/- 0.40, which is what using the full width produces.
    DifferentialDrive drive{geometry()};
    const WheelSpeeds w = drive.to_wheels({0.0F, 1.0F});
    CHECK_NEAR(w.left_mps, -0.20, 1e-6);
    CHECK_NEAR(w.right_mps, 0.20, 1e-6);
}

void test_kinematics_round_trip()
{
    CASE("to_wheels and to_twist invert each other below saturation");
    // Sweeping both axes together is what makes this meaningful: a wrong track
    // convention that is shared by both directions cancels for pure translation
    // and only shows up once yaw is involved.
    DifferentialDrive drive{geometry()};

    for (int i = -5; i <= 5; ++i)
    {
        for (int j = -5; j <= 5; ++j)
        {
            const Twist in{static_cast<float>(i) * 0.1F, static_cast<float>(j) * 0.2F};
            const Twist out = drive.to_twist(drive.to_wheels(in));
            CHECK_NEAR(out.linear_mps, in.linear_mps, 1e-5);
            CHECK_NEAR(out.angular_rps, in.angular_rps, 1e-5);
        }
    }
}

void test_saturation_preserves_curvature()
{
    CASE("saturation costs speed, not path curvature");
    DifferentialDrive drive{geometry()};

    // Ask for far more than the drivetrain can give, on an arc.
    const Twist request{5.0F, 4.0F};
    const WheelSpeeds w = drive.to_wheels(request);

    CHECK(std::fabs(w.left_mps) <= 1.20F + 1e-6F);
    CHECK(std::fabs(w.right_mps) <= 1.20F + 1e-6F);

    // The realised twist must point along the same arc: v/omega unchanged.
    const Twist out = drive.to_twist(w);
    const double requested_radius =
        static_cast<double>(request.linear_mps / request.angular_rps);
    const double realised_radius = static_cast<double>(out.linear_mps / out.angular_rps);
    CHECK_NEAR(realised_radius, requested_radius, 1e-4);

    // ...and it must actually be slower than the request.
    CHECK(std::fabs(out.linear_mps) < std::fabs(request.linear_mps));
}

void test_max_angular_headroom()
{
    CASE("available yaw rate shrinks as forward speed uses up the wheels");
    DifferentialDrive drive{geometry()};
    CHECK_NEAR(drive.max_angular_at(0.0F), 1.20 / 0.20, 1e-5);
    CHECK_NEAR(drive.max_angular_at(1.20F), 0.0, 1e-6);
    CHECK_NEAR(drive.max_angular_at(2.0F), 0.0, 1e-6);
}

void test_odometry_straight_line()
{
    CASE("driving straight advances x and leaves yaw alone");
    DifferentialDrive drive{geometry()};
    Odometry odom{drive};

    // 1 m/s for 1 s, in 10 ms steps.
    for (int i = 0; i < 100; ++i)
    {
        odom.update({1.0F, 1.0F}, 0.01F);
    }

    CHECK_NEAR(odom.pose().x_m, 1.0, 1e-3);
    CHECK_NEAR(odom.pose().y_m, 0.0, 1e-6);
    CHECK_NEAR(odom.pose().yaw_rad, 0.0, 1e-6);
    CHECK_NEAR(odom.pose().twist.linear_mps, 1.0, 1e-6);
}

void test_odometry_spin_in_place()
{
    CASE("spinning on the spot changes yaw and not position");
    DifferentialDrive drive{geometry()};
    Odometry odom{drive};

    // +1 rad/s for 1 s: wheels at -/+ 0.2 m/s.
    for (int i = 0; i < 1000; ++i)
    {
        odom.update({-0.20F, 0.20F}, 0.001F);
    }

    CHECK_NEAR(odom.pose().yaw_rad, 1.0, 1e-3);
    CHECK_NEAR(odom.pose().x_m, 0.0, 1e-6);
    CHECK_NEAR(odom.pose().y_m, 0.0, 1e-6);
}

void test_odometry_quarter_circle()
{
    CASE("a constant-curvature arc lands where geometry says it should");
    DifferentialDrive drive{geometry()};
    Odometry odom{drive};

    // v = 1 m/s, omega = 1 rad/s gives a 1 m radius. A quarter turn should end
    // at (1, 1) facing +Y. This is the case where the straight-line
    // approximation is visibly biased towards the inside of the turn.
    const Twist twist{1.0F, 1.0F};
    const WheelSpeeds wheels = drive.to_wheels(twist);

    const float quarter_turn_s = (kPi / 2.0F);
    const int steps = 1570;
    const float dt = quarter_turn_s / static_cast<float>(steps);

    for (int i = 0; i < steps; ++i)
    {
        odom.update(wheels, dt);
    }

    CHECK_NEAR(odom.pose().x_m, 1.0, 2e-3);
    CHECK_NEAR(odom.pose().y_m, 1.0, 2e-3);
    CHECK_NEAR(odom.pose().yaw_rad, static_cast<double>(kPi) / 2.0, 2e-3);
}

void test_odometry_yaw_stays_wrapped()
{
    CASE("yaw is wrapped rather than growing without bound");
    DifferentialDrive drive{geometry()};
    Odometry odom{drive};

    // Twenty full turns.
    for (int i = 0; i < 20000; ++i)
    {
        odom.update({-0.20F, 0.20F}, 0.0063F);
    }

    CHECK(odom.pose().yaw_rad > -kPi);
    CHECK(odom.pose().yaw_rad <= kPi);
}

void test_odometry_rejects_bad_dt()
{
    CASE("a zero or negative time step is ignored");
    DifferentialDrive drive{geometry()};
    Odometry odom{drive};

    odom.update({1.0F, 1.0F}, 0.0F);
    CHECK_NEAR(odom.pose().x_m, 0.0, 1e-9);

    odom.update({1.0F, 1.0F}, -0.01F);
    CHECK_NEAR(odom.pose().x_m, 0.0, 1e-9);
}

void test_wrap_angle()
{
    CASE("wrap_angle maps onto (-pi, pi]");
    CHECK_NEAR(wrap_angle(0.0F), 0.0, 1e-6);
    CHECK_NEAR(wrap_angle(kPi), kPi, 1e-5);
    CHECK_NEAR(wrap_angle(-kPi), kPi, 1e-5);
    CHECK_NEAR(wrap_angle(3.0F * kPi), kPi, 1e-5);
    CHECK_NEAR(wrap_angle(1.5F * kPi), -0.5 * static_cast<double>(kPi), 1e-5);
    CHECK_NEAR(wrap_angle(-1.5F * kPi), 0.5 * static_cast<double>(kPi), 1e-5);
}

void test_slew_accelerates_and_decelerates()
{
    CASE("the ramp respects separate accel and decel limits");
    SlewLimiter limiter{1.0F, 4.0F};  // 1 m/s^2 up, 4 m/s^2 down

    // 100 ms at 1 m/s^2 is 0.1 m/s.
    CHECK_NEAR(limiter.update(1.0F, 0.1F), 0.1, 1e-6);
    CHECK_NEAR(limiter.update(1.0F, 0.1F), 0.2, 1e-6);

    // Braking is allowed to be four times harder: a 0.4 m/s step in 100 ms,
    // which is more than enough to bring 0.2 m/s to rest in one call.
    CHECK_NEAR(limiter.update(0.0F, 0.1F), 0.0, 1e-6);
}

void test_slew_reaches_target_exactly()
{
    CASE("the ramp settles on the target without overshooting");
    SlewLimiter limiter{2.0F, 2.0F};
    for (int i = 0; i < 100; ++i)
    {
        limiter.update(0.75F, 0.01F);
    }
    CHECK_NEAR(limiter.current(), 0.75, 1e-6);

    // And stays there.
    CHECK_NEAR(limiter.update(0.75F, 0.01F), 0.75, 1e-6);
}

void test_slew_reversal_uses_decel_limit()
{
    CASE("reversing through zero is governed by the decel limit");
    // Moving from -1.0 towards 0 is deceleration even though the number is
    // increasing. Getting that backwards would let the robot brake at the
    // acceleration limit whenever it was travelling in reverse.
    SlewLimiter limiter{0.5F, 5.0F};
    limiter.reset(-1.0F);

    // 100 ms at the decel limit is 0.5, not 0.05.
    CHECK_NEAR(limiter.update(0.0F, 0.1F), -0.5, 1e-6);
}

void test_slew_reset_bypasses_ramp()
{
    CASE("reset jumps straight to the value");
    SlewLimiter limiter{0.1F, 0.1F};
    limiter.reset(3.0F);
    CHECK_NEAR(limiter.current(), 3.0, 1e-6);
    limiter.reset(0.0F);
    CHECK_NEAR(limiter.current(), 0.0, 1e-6);
}

}  // namespace

int main()
{
    std::printf("control\n");
    test_straight_line_has_no_differential();
    test_spin_uses_half_track();
    test_kinematics_round_trip();
    test_saturation_preserves_curvature();
    test_max_angular_headroom();
    test_odometry_straight_line();
    test_odometry_spin_in_place();
    test_odometry_quarter_circle();
    test_odometry_yaw_stays_wrapped();
    test_odometry_rejects_bad_dt();
    test_wrap_angle();
    test_slew_accelerates_and_decelerates();
    test_slew_reaches_target_exactly();
    test_slew_reversal_uses_decel_limit();
    test_slew_reset_bypasses_ramp();
    return auk::test::summary("control");
}
