# Architecture

## The shape of it

```
        link bytes            buttons, bumpers            (future producers)
             |                       |                            |
             v                       v                            v
      +--------------------------------------------------------------------+
      |                            CommandBus                              |
      |        High (reject)   |   Normal (newest)   |   Low (newest)      |
      +--------------------------------------------------------------------+
                                     |
                                     v
                          Robot::dispatch_commands
                                     |
                                     v
                            requested twist, lamp,
                            e-stop request
                                     |
      hardware inputs ---------------+
      (e-stop, bumpers,              |
       link state, battery)          v
                            +------------------+
                            |    Interlock     |   <-- the only thing that
                            +------------------+       decides what is allowed
                                     |
                                     v
                             permitted twist
                                     |
                                     v
                            +------------------+
                            |  Slew limiters   |
                            +------------------+
                                     |
                                     v
                       DifferentialDrive::to_wheels
                                     |
                                     v
                            hal::Drivetrain
                                     |
    measured wheel speeds  <---------+
             |
             v
        Odometry ---> pose ---> TelemetrySnapshot ---> hal::Link
```

Two properties are worth stating explicitly, because everything else follows
from them:

**Exactly one function commands the drivetrain.** Every producer — the link
parser, a button interrupt, an autonomy layer you add later — posts a `Command`
and returns. `Robot::run_control` is the only code that touches
`hal::Drivetrain`. When somebody asks "what could make this thing move", there is
one honest answer and it fits on a screen.

**Permission is computed, not assumed.** The interlock takes a struct of inputs
and returns what is allowed. It does not read globals, it does not call hardware,
and it holds no state beyond the current fault set. That is what makes its truth
table testable — every combination can be constructed directly, including the
ones that are awkward to stage on a real machine.

## Execution model

There is no RTOS. `Robot::step()` is called as fast as the main loop can manage,
and each activity is gated by its own `Deadline`:

| Activity | Period | Why |
| --- | --- | --- |
| Link service | every call | Input that is not drained promptly overflows a receive buffer, and a dropped byte costs a whole command line |
| Control | 10 ms | Fast enough for the ramps and odometry to be smooth; slow enough to leave headroom |
| Telemetry | 50 ms | 20 Hz is plenty for a supervisor, and it keeps the link quiet |
| Watchdog kick | end of every call | See below |

One context means no priority inversion, no stack sizing, no locks, and no
scheduler to misconfigure. The price is that every activity must be non-blocking
and short. That is a real constraint — you cannot `delay()` anywhere — but on a
machine this size it is also what makes the timing analysable by reading the
code rather than by instrumenting it.

If you outgrow this, the natural next step is to keep the same decomposition and
put the control loop in one RTOS task with the link parser in another, feeding
the same bus. The bus is already the thread boundary.

## One control cycle, in order

From `Robot::step()`:

1. **Drain the link.** Bytes are reassembled into lines, parsed, and posted as
   commands. Malformed input is rejected, not guessed at.
2. **If the control deadline has expired:**
   1. **Drain the safety lane completely.** Every element is a discrete event —
      a bumper edge, an e-stop press — and handling one per cycle would let a
      press and its release sit in the queue across cycles.
   2. **Take one command each from the normal and low lanes.** These are
      level-like: the newest setpoint supersedes the last, so there is nothing
      to gain from draining, and bounded work keeps the period predictable.
   3. **Sample hardware** — digital inputs and the power monitor — as one
      coherent snapshot.
   4. **Update the battery gauge.**
   5. **Evaluate the interlock** against every input at once.
   6. **If motion is allowed:** release the drivetrain if it was stopped, ramp
      the permitted twist, convert to wheel speeds, command them.
      **Otherwise:** cut torque at the hardware and zero the ramps, so that
      motion resumes from standstill rather than from the demand that was in
      flight when the fault appeared.
   7. **Integrate odometry** from *measured* wheel speeds, so a stalled or
      slipping wheel shows up as the robot not moving.
3. **If the telemetry deadline has expired,** assemble a snapshot and write it.
4. **Kick the watchdog.**

Step 4 is last on purpose. Reaching it means a whole iteration completed;
anything that wedges earlier stops kicking and the part resets. A kick at the top
of the loop — or worse, from a timer interrupt — keeps the robot alive while its
control loop is dead, which is precisely the case the watchdog exists to catch.

## Modules

### `core`

`RingBuffer<T, N, Policy>` — fixed capacity, member storage, no allocation. The
count is tracked explicitly rather than derived from the two indices, because the
derived form needs a sacrificial slot or a wrap-parity bit to tell full from
empty, and getting that subtly wrong gives you a buffer that reports itself empty
while holding a full set of unread elements.

The overflow policy is a type parameter rather than a runtime flag, so the
decision is visible at the declaration:

- `Reject` — refuse the new element. For anything where every element matters.
- `DropOldest` — retire the oldest. For anything where only recency matters.

`Deadline` — wrap-safe periodic timing. Never reads a clock itself; it is always
given `now`. That keeps every user testable without faking time globally, and
makes the cost of reading the clock visible at the call site.

`arm_periodic()` restarts from when the deadline was *due*, not from now, so a
task that is habitually a millisecond late does not silently become a slower
task. If it has fallen more than a whole period behind it resynchronises instead
of trying to catch up, because catching up means firing repeatedly with no gap.

`CommandBus` — three lanes, capacities chosen from what each is for. The high
lane rejects; the others overwrite. A rejection on the high lane latches
`safety_lane_overflowed()`, which the interlock treats as a hard fault: if a
safety event was dropped we cannot know what it was.

### `control`

`DifferentialDrive` — inverse and forward kinematics from one `DriveGeometry`.
Saturation scales both wheels by the same factor, so the arc survives and only
speed is lost; clamping them independently would change the turn radius, so a
robot asked to drive fast along a gentle curve would straighten out as it
saturated.

`Odometry` — pose from measured wheel speeds over an explicit `dt`. Uses the
exact constant-curvature arc rather than the straight-line approximation, because
the approximation's error is systematically biased to the inside of the turn and
therefore does not average out. Yaw is wrapped every update.

`SlewLimiter` — separate acceleration and deceleration limits, because they are
different engineering questions: how fast you may add energy is about traction
and comfort, how fast you may remove it is about stopping. Which limit applies
depends on whether the *magnitude* of the demand is growing, not on the sign of
the change — going from −1 m/s towards 0 is deceleration.

### `safety`

`Interlock` — the whole permission model. Starts faulted and stays that way until
it has positively seen a healthy input set. Reports every active fault rather
than the first one found, because an operator looking at a stationary robot needs
the whole picture.

Bumpers are handled differently from other faults: they remove only the linear
component driving into them, leaving reverse and rotation available. A robot that
has to be carried off its own bumper is worse than one that will not drive into
things.

### `power`

`BatteryGauge` — voltage-lookup state of charge with IR compensation and a
monotonicity rule. Deliberately not coulomb counting, which needs a
well-characterised current-sensor zero and an integrator reset against a known
full charge; an uncalibrated one drifts without bound, while voltage lookup is
cruder and cannot.

### `hal`

Pure interfaces, no platform headers. The interfaces are narrow on purpose: a HAL
that exposes "the CAN bus" invites the application to know about CAN, while one
that exposes "wheel speeds" does not.

`Drivetrain::emergency_stop()` is separate from `set_wheel_speeds({0,0})`.
A zero setpoint asks a controller that may itself be wedged to please stop; the
emergency stop removes its ability to drive. When those are the same call, the
e-stop is only as reliable as the thing it is overriding.

### `app`

`Robot` owns the loop. `telemetry` is the wire codec — line-oriented text,
because at a few hundred bytes at 20 Hz the bandwidth saving from a packed struct
is irrelevant while the cost is real: silent breakage when the two ends are built
from different headers, and a wire you cannot debug without custom tooling.

## Testing

`tests/` has four binaries:

- `test_core` — ring buffer policies, deadline arithmetic including the 32-bit
  wrap, command bus lane behaviour.
- `test_control` — kinematics round-trip over a swept grid, saturation
  preserving curvature, odometry against closed-form geometry, slew limiting.
- `test_safety` — the interlock truth table and the battery gauge.
- `test_integration` — the real firmware against the simulated plant.

The integration tests are the ones that catch wiring mistakes: a module that is
correct in isolation but never called, or called in the wrong order. Every
scenario in there would otherwise need a robot, a clear floor and a willing
volunteer.

Time in the simulator is virtual and advanced explicitly, so a scenario runs
deterministically and as fast as the host will go. Nothing sleeps; the whole
suite is about 20 ms.
