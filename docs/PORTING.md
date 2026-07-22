# Porting to real hardware

Everything under `include/` and `src/` is already portable: no platform headers,
no allocation after startup, no exceptions, no RTTI. Porting means implementing
seven interfaces from `include/auk/hal/hal.hpp` and writing a `main`.

## What you implement

| Interface | Typically |
| --- | --- |
| `Clock` | A free-running millisecond counter — SysTick, a hardware timer, or your framework's `millis()` |
| `Drivetrain` | Motor controller commands and encoder feedback, in **m/s at the wheel** |
| `InputPanel` | GPIO reads for e-stop and bumpers, debounced |
| `PowerMonitor` | ADC reads for pack voltage and current, scaled to volts and amps |
| `Lamp` | One GPIO or PWM output |
| `Link` | Non-blocking byte I/O — USB CDC, UART, RS-485, TCP |
| `Watchdog` | The independent watchdog peripheral |

Then:

```cpp
int main()
{
    MyBoard board;                 // your seven implementations
    auk::app::Robot robot{board.hal(), make_config()};

    if (!robot.begin())
    {
        panic("incomplete board");  // do not proceed
    }

    for (;;)
    {
        robot.step();
    }
}
```

`begin()` returning false means a HAL pointer is null. Do not carry on — running
with a null drivetrain or a null watchdog is strictly worse than not running.

## Order to bring it up

Doing this out of order is how bring-up turns into a week. Each step below is
verifiable before the next one can hurt you.

**1. `Clock`, and nothing else.** Return a counter from `now_ms()`. Verify it
advances at the right rate over a minute against a stopwatch — a clock that is
20% fast makes every subsequent timing observation a lie, and you will chase it
into the control loop.

**2. `Watchdog`.** Implement `start()` and `kick()`, then deliberately hang the
loop and confirm the part resets. A watchdog you have not seen fire is not a
watchdog.

**3. `Link`.** Implement read and write. Send `L 1` and watch for the lamp
command to arrive. At this point you have a debug channel, which makes everything
after it far easier.

**4. `InputPanel`.** Debounce in your implementation, not in the application —
the application assumes a clean level. Confirm that pressing the e-stop makes
`faults=` in the telemetry show the e-stop bit.

**5. `PowerMonitor`.** Get the scaling right before you trust anything. Compare
against a meter at two well-separated points, not one; a single point cannot
distinguish a gain error from an offset.

**6. `Drivetrain`, wheels off the ground.** This is the one that can hurt
somebody. Chock the robot or put it on blocks. Verify sign conventions:

- `V 0.2 0.0` — both wheels forward.
- `V 0.0 0.5` — left wheel backward, right wheel forward (positive yaw is
  counter-clockwise seen from above).
- `measured_speeds()` must have the same signs as the commands. If a wheel reads
  negative while driving forward, fix it in the HAL, not by negating something
  in the application.

**7. Odometry, on the ground.** Drive a measured 5 m and compare `x=` in the
telemetry against a tape measure. If it is off by a consistent percentage, your
wheel radius is wrong — measure the effective rolling radius under load rather
than trusting the datasheet.

Then drive a measured 360° turn on the spot and check `yaw=` returns to where it
started. **If yaw is out by a factor of two, you have the track-width convention
backwards** — `track_width_m` is the full track, centre to centre, and the
kinematics halve it internally.

## Configuration

`app::RobotConfig` holds everything machine-specific. Measure, do not assume:

```cpp
config.geometry.track_width_m = 0.40F;   // full track, centre to centre
config.geometry.wheel_radius_m = 0.08F;  // effective rolling radius, under load
config.geometry.max_wheel_mps = 1.20F;   // what the drivetrain actually delivers

config.accel_limit = 0.8F;               // m/s^2
config.decel_limit = 2.0F;               // m/s^2, may be much higher than accel
config.angular_accel_limit = 2.5F;       // rad/s^2
config.angular_decel_limit = 5.0F;

config.control_period_ms = 10;
config.telemetry_period_ms = 50;
config.watchdog_timeout_ms = 100;
```

The battery curve is per pack chemistry. Get it by discharging a known-good pack
at a low, constant current and recording open-circuit voltage against
amp-hours removed. Seven points is enough — the curve is flat through the middle
and steep at both ends, which is why the gauge interpolates a table rather than
fitting a polynomial.

## Choosing the periods

`control_period_ms` sets the slew resolution and the odometry step. 10 ms is a
reasonable default. Going faster costs CPU and buys little unless your drivetrain
is unusually responsive; going much slower makes the ramps visibly steppy.

`watchdog_timeout_ms` must exceed your worst-case iteration, not your typical
one. Measure it: instrument `step()` with your clock, log the maximum over an
hour of real work, and leave a factor of three. The default of 100 ms against a
10 ms period is deliberately generous.

## Things that will bite

**`measured_speeds()` must be in m/s at the wheel**, not RPM, not encoder counts.
Doing that conversion in the HAL keeps the gear ratio and the counts-per-rev in
one place. Returning RPM here and getting away with it in a straight line is a
classic — the error is a constant factor, so it looks like a calibration problem
right up until you try to close a loop.

**`emergency_stop()` must actually remove torque.** A driver disable pin, a
contactor, a safe-torque-off input. If you implement it as "command zero speed",
your emergency stop is only as reliable as the controller you are trying to
override, which is the thing you did not trust.

**`Link::read` and `write` must not block.** They are called from the control
loop. A blocking write on a full UART stalls everything and trips the watchdog —
which is at least a loud failure, but not the one you wanted.

**Debounce your bumpers.** The interlock reacts to the level it is given, and an
undebounced bumper chatters between faulted and clear at whatever rate the
contacts bounce.

**The interlock is not a safety system.** It runs on the same processor as the
code it is protecting, so a fault that takes out the loop takes out the interlock
too. A real machine needs a hardware e-stop circuit that removes motor power
without asking software for permission. If you need a rating, that circuit gets
assessed against ISO 13849 or IEC 62061; nothing here substitutes for it.

## Cross-compiling

The library has no build-system dependency on the host toolchain. With a CMake
toolchain file:

```sh
cmake -B build-arm -DCMAKE_TOOLCHAIN_FILE=my-arm.cmake
cmake --build build-arm --target auk
```

`auk-sim`, `auk-sim-demo` and the tests are host-only — they use `<string>` and
`<deque>` — so build the `auk` target alone when cross-compiling.

Keep running the host tests in CI regardless of what you deploy to. They exercise
the same source, they run in milliseconds, and they cover cases you cannot
reproduce on a bench.
