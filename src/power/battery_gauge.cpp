// SPDX-License-Identifier: MIT
#include "auk/power/battery_gauge.hpp"

#include <cmath>

namespace auk::power
{

BatteryGauge::BatteryGauge(const BatteryConfig& config) noexcept : config_{config} {}

float BatteryGauge::lookup_soc(float ocv) const noexcept
{
    if (config_.curve == nullptr || config_.curve_length == 0)
    {
        return 1.0F;
    }

    const SocPoint* curve = config_.curve;
    const std::size_t n = config_.curve_length;

    // Clamp outside the characterised range rather than extrapolating. The curve
    // goes vertical at both ends, so extrapolation there produces confidently
    // wrong numbers from tiny voltage errors.
    if (ocv <= curve[0].open_circuit_volts)
    {
        return curve[0].state_of_charge;
    }
    if (ocv >= curve[n - 1].open_circuit_volts)
    {
        return curve[n - 1].state_of_charge;
    }

    for (std::size_t i = 1; i < n; ++i)
    {
        if (ocv <= curve[i].open_circuit_volts)
        {
            const float v0 = curve[i - 1].open_circuit_volts;
            const float v1 = curve[i].open_circuit_volts;
            const float s0 = curve[i - 1].state_of_charge;
            const float s1 = curve[i].state_of_charge;

            const float span = v1 - v0;
            if (span <= 0.0F)
            {
                return s1;  // duplicate or unsorted entry; do not divide by it
            }

            return s0 + (s1 - s0) * ((ocv - v0) / span);
        }
    }

    return curve[n - 1].state_of_charge;
}

void BatteryGauge::update(float terminal_volts, float current_a) noexcept
{
    // Undo the IR drop to recover open-circuit voltage. Under discharge
    // (positive current) the terminal reads low, so the correction adds.
    ocv_ = terminal_volts + (current_a * config_.internal_resistance_ohm);

    const float measured = lookup_soc(ocv_);

    if (!has_reading_)
    {
        soc_ = measured;
        has_reading_ = true;
        return;
    }

    if (measured <= soc_)
    {
        soc_ = measured;
        rise_samples_ = 0;
        return;
    }

    // The measurement went up. Small rises are noise or recovery from a load
    // transient and are ignored; a large one, sustained across several samples,
    // is a charge or a pack swap and is accepted. Requiring persistence is what
    // keeps a single spike from resetting the gauge.
    if ((measured - soc_) >= kPackChangeThreshold)
    {
        ++rise_samples_;
        if (rise_samples_ >= kPackChangeSamples)
        {
            soc_ = measured;
            rise_samples_ = 0;
        }
    }
    else
    {
        rise_samples_ = 0;
    }
}

}  // namespace auk::power
