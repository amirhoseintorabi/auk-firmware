// SPDX-License-Identifier: MIT
#include "auk/control/slew_limiter.hpp"

#include <cmath>

namespace auk::control
{

float SlewLimiter::update(float target, float dt_s) noexcept
{
    if (!(dt_s > 0.0F))
    {
        return current_;
    }

    const float delta = target - current_;
    if (delta == 0.0F)
    {
        return current_;
    }

    // Which limit applies depends on whether the magnitude of the demand is
    // growing or shrinking -- not on the sign of the change. Moving from -1 m/s
    // towards 0 is a *deceleration* even though the value is increasing, and
    // getting that backwards means the robot brakes at the acceleration limit
    // whenever it is reversing.
    const bool speeding_up = std::fabs(target) > std::fabs(current_);

    // A demand that crosses zero does both: it decelerates to a stop, then
    // accelerates the other way. Handling it in one step with a single limit
    // would let the more permissive of the two govern a transition it has no
    // business governing, so the crossing is treated as deceleration and the
    // rest happens on the following call.
    const bool crosses_zero = (current_ * target) < 0.0F;

    const float limit = (speeding_up && !crosses_zero) ? accel_limit_ : decel_limit_;

    if (!(limit > 0.0F))
    {
        // Disabled in this direction: pass the demand straight through.
        current_ = target;
        return current_;
    }

    const float max_step = limit * dt_s;
    if (std::fabs(delta) <= max_step)
    {
        current_ = target;
    }
    else
    {
        current_ += (delta > 0.0F) ? max_step : -max_step;
    }

    return current_;
}

}  // namespace auk::control
