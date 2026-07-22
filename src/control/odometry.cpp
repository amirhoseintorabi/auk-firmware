// SPDX-License-Identifier: MIT
#include "auk/control/odometry.hpp"

#include <cmath>

namespace auk::control
{
namespace
{

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 2.0F * kPi;

/// Below this yaw rate the arc and the straight line agree to well within float
/// precision, and the arc form divides by the yaw rate. Switching to the
/// straight-line form here avoids the singularity without introducing error
/// anyone can measure.
constexpr float kStraightLineYawEpsilon = 1e-6F;

}  // namespace

float wrap_angle(float radians) noexcept
{
    // fmod keeps this O(1) regardless of how far out of range the input is; a
    // while-loop version is fine for normal use and pathological if it is ever
    // handed a large angle.
    float wrapped = std::fmod(radians + kPi, kTwoPi);
    if (wrapped <= 0.0F)
    {
        wrapped += kTwoPi;
    }
    return wrapped - kPi;
}

void Odometry::update(const WheelSpeeds& wheels, float dt_s) noexcept
{
    if (!(dt_s > 0.0F))
    {
        // Zero, negative or NaN. A negative step would drive the pose backwards
        // along its own history, which is never what a caller means; reject it
        // rather than integrate nonsense.
        return;
    }

    const Twist twist = drive_.to_twist(wheels);
    pose_.twist = twist;

    const float yaw0 = pose_.yaw_rad;
    const float dtheta = twist.angular_rps * dt_s;

    if (std::fabs(twist.angular_rps) < kStraightLineYawEpsilon)
    {
        const float distance = twist.linear_mps * dt_s;
        pose_.x_m += distance * std::cos(yaw0);
        pose_.y_m += distance * std::sin(yaw0);
    }
    else
    {
        // Constant-curvature arc over the step. The instantaneous centre of
        // rotation sits at radius v/omega abeam the robot, and the chord is
        // obtained from the difference of the two heading sines/cosines.
        const float radius = twist.linear_mps / twist.angular_rps;
        const float yaw1 = yaw0 + dtheta;

        pose_.x_m += radius * (std::sin(yaw1) - std::sin(yaw0));
        pose_.y_m -= radius * (std::cos(yaw1) - std::cos(yaw0));
    }

    pose_.yaw_rad = wrap_angle(yaw0 + dtheta);
}

void Odometry::set_pose(float x_m, float y_m, float yaw_rad) noexcept
{
    pose_.x_m = x_m;
    pose_.y_m = y_m;
    pose_.yaw_rad = wrap_angle(yaw_rad);
}

}  // namespace auk::control
