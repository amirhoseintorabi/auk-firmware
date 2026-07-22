// SPDX-License-Identifier: MIT
#pragma once

#include <cstddef>
#include <cstdint>

namespace auk::power
{

/// A point on the pack's open-circuit-voltage to state-of-charge curve.
struct SocPoint
{
    float open_circuit_volts{0.0F};
    float state_of_charge{0.0F};  ///< 0.0 to 1.0
};

struct BatteryConfig
{
    /// Pack internal resistance in ohms, used to back out open-circuit voltage
    /// from the loaded terminal voltage. Measure it: a step change in load
    /// current divided into the resulting step in terminal voltage. The
    /// datasheet figure is for a new cell at 25 degrees and will be optimistic
    /// for anything in service.
    float internal_resistance_ohm{0.02F};

    /// Below this state of charge the interlock treats the pack as critical.
    float critical_soc{0.10F};

    /// Below this, report low but keep running.
    float low_soc{0.25F};

    /// Discharge curve, ascending by voltage. Interpolated linearly between
    /// points and clamped outside the range.
    ///
    /// A lookup table rather than a fitted polynomial because the curve is
    /// flat through most of the usable range and steep at both ends: a
    /// polynomial that fits the knees will ripple across the plateau, which is
    /// precisely where you want the reading to be steady.
    const SocPoint* curve{nullptr};
    std::size_t curve_length{0};
};

/// State of charge from terminal voltage and current.
///
/// Two things this gauge does that a naive one does not:
///
///  - **It compensates for load before looking anything up.** Terminal voltage
///    under load is depressed by I*R, so reading the curve directly makes the
///    pack look empty every time the robot accelerates and full again whenever
///    it coasts. The reported charge then swings by tens of percent with no
///    change in the actual pack.
///
///  - **It is monotonic in normal operation.** Charge is not permitted to rise
///    while discharging, because every source of noise in the measurement is
///    symmetric while the underlying quantity is not. Without this the display
///    jitters upward constantly and operators stop believing it. A genuine
///    increase -- a charge, or a pack swap -- is detected as a sustained jump
///    and allowed through.
///
/// Deliberately *not* coulomb counting. That needs a current sensor with a
/// well-characterised zero offset and an integrator that is reset against a
/// known full charge, and an uncalibrated one drifts without bound. Voltage
/// lookup is cruder and cannot drift.
class BatteryGauge
{
public:
    explicit BatteryGauge(const BatteryConfig& config) noexcept;

    /// Feed one sample. `current_a` is positive when discharging.
    void update(float terminal_volts, float current_a) noexcept;

    float state_of_charge() const noexcept { return soc_; }
    float open_circuit_volts() const noexcept { return ocv_; }

    bool is_low() const noexcept { return soc_ <= config_.low_soc; }
    bool is_critical() const noexcept { return soc_ <= config_.critical_soc; }

    /// True once a sample has been taken. Before that, `state_of_charge()`
    /// returns 1.0, and callers should not act on it -- reporting a full pack
    /// is the safe default for display but the wrong basis for a decision.
    bool has_reading() const noexcept { return has_reading_; }

private:
    float lookup_soc(float ocv) const noexcept;

    BatteryConfig config_;
    float soc_{1.0F};
    float ocv_{0.0F};
    bool has_reading_{false};

    /// A rise larger than this, sustained, is taken as a real change in the pack
    /// rather than measurement noise.
    static constexpr float kPackChangeThreshold = 0.15F;
    static constexpr int kPackChangeSamples = 8;
    int rise_samples_{0};
};

}  // namespace auk::power
