// SPDX-License-Identifier: MIT
#pragma once

#include <cstdint>

#include "auk/control/differential_drive.hpp"

namespace auk::control
{

/// Planar pose plus the velocity that produced it.
struct Pose2D
{
    float x_m{0.0F};
    float y_m{0.0F};
    float yaw_rad{0.0F};  ///< wrapped to (-pi, pi]
    Twist twist{};        ///< body velocity at the last update
};

/// Dead-reckoned pose from wheel velocities.
///
/// Integration is explicit in the caller's time step rather than read from a
/// clock inside `update`, for two reasons. It makes the class testable with
/// exact time steps, and it stops the integrator inheriting whatever jitter the
/// scheduler has: a pose integrator that measures its own dt against wall time
/// accumulates every scheduling hiccup as real displacement.
///
/// Two things this deliberately does *not* do:
///
///  - It does not integrate per sensor message. Encoder frames arrive at a rate
///    that depends on bus load, and advancing the pose once per frame silently
///    ties the integration step to how busy the bus is.
///  - It does not let yaw grow without bound. Wrapping to (-pi, pi] on every
///    update keeps the argument to sin/cos small; letting it run means that
///    after a few hours of spinning, float precision near the accumulated
///    magnitude is coarser than the per-step increment and the pose quietly
///    stops updating.
class Odometry
{
public:
    explicit Odometry(const DifferentialDrive& drive) noexcept : drive_{drive} {}

    /// Advance the pose by `dt_s` at the measured wheel speeds.
    ///
    /// Uses the exact arc for a constant-curvature step rather than the
    /// straight-line approximation. Over a 10 ms step at plausible speeds the
    /// difference is tiny, but it is systematically biased -- always to the
    /// inside of the turn -- so it does not average out, and on a robot that
    /// spends its life following curved aisles it shows up as a steady radial
    /// drift.
    void update(const WheelSpeeds& wheels, float dt_s) noexcept;

    const Pose2D& pose() const noexcept { return pose_; }

    /// Return to the origin. Velocity is cleared too, so a reset cannot leave a
    /// stale twist being reported as current.
    void reset() noexcept { pose_ = Pose2D{}; }

    /// Move the origin without discarding the current velocity estimate, for
    /// when an external fix (a fiducial, a scan match) corrects the pose.
    void set_pose(float x_m, float y_m, float yaw_rad) noexcept;

private:
    DifferentialDrive drive_;
    Pose2D pose_{};
};

/// Wrap an angle to (-pi, pi].
float wrap_angle(float radians) noexcept;

}  // namespace auk::control
