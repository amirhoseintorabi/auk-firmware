// SPDX-License-Identifier: MIT
#pragma once

namespace auk::control
{

/// Rate limiter for a single scalar setpoint.
///
/// A motion controller that passes commands straight through will happily ask
/// for a step from full forward to full reverse, because nothing upstream knows
/// what the drivetrain can survive. This bounds how fast the demand may change,
/// which is the cheapest form of mechanical protection there is -- no model, no
/// tuning, one constant per axis.
///
/// Acceleration and deceleration are separate limits. They are not the same
/// engineering quantity: how hard you may *add* energy is a traction and comfort
/// question, while how hard you may *remove* it is a safety question, and a
/// stopping ramp should usually be allowed to be much more aggressive than a
/// starting one. Collapsing them into a single "max accel" is a common
/// simplification that quietly makes emergency stops gentler than they should
/// be.
class SlewLimiter
{
public:
    SlewLimiter() = default;

    /// Units are per second: a limiter on m/s takes m/s^2.
    /// A non-positive limit disables that direction.
    SlewLimiter(float accel_limit, float decel_limit) noexcept
        : accel_limit_{accel_limit}, decel_limit_{decel_limit}
    {
    }

    /// Step the output towards `target` and return the new value.
    float update(float target, float dt_s) noexcept;

    /// Jump straight to `value`, bypassing the ramp.
    ///
    /// For emergency stop and for re-syncing after the output has been
    /// overridden downstream: resuming a ramp from a stale internal value would
    /// make the limiter fight whatever actually happened to the plant.
    void reset(float value = 0.0F) noexcept { current_ = value; }

    float current() const noexcept { return current_; }

    void set_limits(float accel_limit, float decel_limit) noexcept
    {
        accel_limit_ = accel_limit;
        decel_limit_ = decel_limit;
    }

private:
    float current_{0.0F};
    float accel_limit_{0.0F};
    float decel_limit_{0.0F};
};

}  // namespace auk::control
