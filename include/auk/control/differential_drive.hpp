// SPDX-License-Identifier: MIT
#pragma once

namespace auk::control
{

/// Commanded or measured motion of the robot body.
struct Twist
{
    float linear_mps{0.0F};   ///< forward, +X, metres per second
    float angular_rps{0.0F};  ///< yaw, +CCW seen from above, radians per second
};

/// Left and right wheel speeds.
struct WheelSpeeds
{
    float left_mps{0.0F};
    float right_mps{0.0F};
};

/// Physical dimensions of a differential-drive base.
struct DriveGeometry
{
    /// Distance between the two wheel contact patches, in metres.
    ///
    /// This is the **full** track width, centre of the left tyre to centre of
    /// the right. The kinematics below divide by two internally to get the
    /// moment arm.
    ///
    /// It is worth being explicit, because "wheel separation" is the single
    /// most reliably confused constant in mobile robotics: the body-to-wheel
    /// relation `v = omega * r` wants the *half* track, and passing the full
    /// width where the half is meant gives you exactly a factor of two in yaw.
    /// The failure is unusually hard to notice, because forward and inverse
    /// kinematics normally share the same mistake -- the robot then reports
    /// precisely the yaw rate it was asked for while turning at half that rate
    /// in the real world, and only closing the loop against an external
    /// reference such as a lidar scan match reveals it.
    float track_width_m{0.4F};

    /// Effective rolling radius, in metres. Measure it under load: a pneumatic
    /// or foam tyre squashes, and the manufacturer's figure is usually a couple
    /// of percent optimistic, which shows up as a slow drift in odometry scale.
    float wheel_radius_m{0.08F};

    /// Largest wheel speed the drivetrain can actually deliver, in m/s. Used to
    /// scale commands that ask for more than the hardware can give.
    float max_wheel_mps{1.5F};
};

/// Inverse and forward kinematics for a two-wheel differential base.
///
/// Stateless and free of hardware dependencies, so it can be exercised
/// exhaustively on a host. `to_wheels` and `to_twist` are exact inverses of each
/// other inside the speed limit, which the test suite asserts over a sweep of
/// inputs -- that round-trip property is what catches a wrong track-width
/// convention, and it only works because both directions are derived here from
/// the same geometry rather than being written out twice.
class DifferentialDrive
{
public:
    explicit DifferentialDrive(const DriveGeometry& geometry) noexcept
        : geometry_{geometry}
    {
    }

    const DriveGeometry& geometry() const noexcept { return geometry_; }

    /// Body twist to wheel speeds.
    ///
    /// If either wheel would exceed `max_wheel_mps`, **both** are scaled by the
    /// same factor. Clamping them independently would change the ratio between
    /// them, which changes the turn radius: a robot asked to drive fast along a
    /// gentle arc would straighten out as it saturated, instead of simply
    /// driving the same arc more slowly. Preserving the ratio means saturation
    /// costs speed, never path.
    WheelSpeeds to_wheels(const Twist& twist) const noexcept;

    /// Wheel speeds to body twist. The inverse of `to_wheels`, ignoring
    /// saturation.
    Twist to_twist(const WheelSpeeds& wheels) const noexcept;

    /// Largest yaw rate achievable while holding `linear_mps`, in rad/s.
    /// Returns 0 if the linear speed alone already saturates the wheels.
    float max_angular_at(float linear_mps) const noexcept;

private:
    DriveGeometry geometry_;
};

}  // namespace auk::control
