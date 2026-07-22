// SPDX-License-Identifier: MIT
#include "auk/safety/interlock.hpp"

namespace auk::safety
{

void Interlock::update(const SafetyInputs& inputs, core::Millis now) noexcept
{
    FaultFlags faults = FaultFlags::None;

    if (inputs.estop_asserted)
    {
        faults |= FaultFlags::EmergencyStop;
    }
    if (inputs.front_bumper_pressed)
    {
        faults |= FaultFlags::FrontBumper;
    }
    if (inputs.rear_bumper_pressed)
    {
        faults |= FaultFlags::RearBumper;
    }
    if (!inputs.link_up)
    {
        faults |= FaultFlags::LinkLost;
    }
    if (inputs.battery_critical)
    {
        faults |= FaultFlags::BatteryCritical;
    }
    if (inputs.safety_queue_overflowed)
    {
        // Not recoverable by observation: once a safety event has been dropped
        // we cannot know what it was, so the robot stays down until a human
        // power-cycles it.
        faults |= FaultFlags::SafetyQueueLost;
    }

    // Checked even when the link reports itself down, so that the reason shown
    // to an operator is complete rather than stopping at the first fault found.
    if (core::elapsed_since(inputs.last_command_ms, now) > kCommandTimeoutMs)
    {
        faults |= FaultFlags::CommandStale;
    }

    faults_ = faults;
}

FaultFlags Interlock::blocking_faults() const noexcept
{
    constexpr auto kBumpers = static_cast<std::uint16_t>(FaultFlags::FrontBumper) |
                              static_cast<std::uint16_t>(FaultFlags::RearBumper);

    // Bumpers restrict a direction rather than stopping the robot, so they are
    // masked out of the "is anything stopping us" question.
    return static_cast<FaultFlags>(static_cast<std::uint16_t>(faults_) &
                                   static_cast<std::uint16_t>(~kBumpers));
}

control::Twist Interlock::permit(const control::Twist& request) const noexcept
{
    if (any(blocking_faults()))
    {
        return control::Twist{};
    }

    control::Twist allowed = request;

    // A pressed bumper removes only the linear component that drives further
    // into it. Yaw is left alone: rotating in place is how an operator gets the
    // robot pointed away from what it hit, and forbidding that turns a nuisance
    // into a recovery job.
    if (has(faults_, FaultFlags::FrontBumper) && allowed.linear_mps > 0.0F)
    {
        allowed.linear_mps = 0.0F;
    }
    if (has(faults_, FaultFlags::RearBumper) && allowed.linear_mps < 0.0F)
    {
        allowed.linear_mps = 0.0F;
    }

    return allowed;
}

}  // namespace auk::safety
