// SPDX-License-Identifier: MIT
#include "auk/control/differential_drive.hpp"

#include <cmath>

namespace auk::control
{
namespace
{

float abs_max(float a, float b) noexcept
{
    return std::fmax(std::fabs(a), std::fabs(b));
}

}  // namespace

WheelSpeeds DifferentialDrive::to_wheels(const Twist& twist) const noexcept
{
    // Moment arm is the half track, not the track. See DriveGeometry.
    const float half_track = 0.5F * geometry_.track_width_m;
    const float differential = twist.angular_rps * half_track;

    WheelSpeeds wheels{};
    wheels.left_mps = twist.linear_mps - differential;
    wheels.right_mps = twist.linear_mps + differential;

    const float peak = abs_max(wheels.left_mps, wheels.right_mps);
    if (peak > geometry_.max_wheel_mps && peak > 0.0F)
    {
        // Scale both wheels together so the ratio -- and therefore the arc the
        // robot follows -- survives saturation.
        const float scale = geometry_.max_wheel_mps / peak;
        wheels.left_mps *= scale;
        wheels.right_mps *= scale;
    }

    return wheels;
}

Twist DifferentialDrive::to_twist(const WheelSpeeds& wheels) const noexcept
{
    const float half_track = 0.5F * geometry_.track_width_m;

    Twist twist{};
    twist.linear_mps = 0.5F * (wheels.left_mps + wheels.right_mps);

    if (half_track > 0.0F)
    {
        twist.angular_rps = 0.5F * (wheels.right_mps - wheels.left_mps) / half_track;
    }

    return twist;
}

float DifferentialDrive::max_angular_at(float linear_mps) const noexcept
{
    const float headroom = geometry_.max_wheel_mps - std::fabs(linear_mps);
    if (headroom <= 0.0F)
    {
        return 0.0F;
    }

    const float half_track = 0.5F * geometry_.track_width_m;
    if (half_track <= 0.0F)
    {
        return 0.0F;
    }

    return headroom / half_track;
}

}  // namespace auk::control
