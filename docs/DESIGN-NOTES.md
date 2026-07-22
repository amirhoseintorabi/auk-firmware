# Design notes

Why each decision went the way it did, and what it costs. The intent is that
somebody disagreeing with a choice can see what it was trading against rather
than having to reconstruct it.

---

## One writer to the drivetrain

**Decision.** Producers post `Command`s. `Robot::run_control` is the only code
that calls `hal::Drivetrain`.

**Why.** The alternative — each module driving the hardware when it has something
to say — is easier to write and much harder to reason about. With five producers
you have five places that can move the robot, five orderings to think about when
two of them disagree, and no single point to put a safety check. Every interlock
then has to be replicated in every producer, and it will not be.

**What it costs.** A layer of indirection, and a small amount of latency: a
command posted just after the dispatch point waits until the next control cycle,
up to 10 ms. For safety events that is bounded and acceptable; if it were not,
the answer would be to shorten the control period, not to let producers reach
past the bus.

---

## Permission defaults to "no"

**Decision.** `Interlock` is constructed with `EmergencyStop | LinkLost` set, and
only clears them once it has seen a healthy input set.

**Why.** The opposite default fails in the worst possible direction. A robot that
permits motion because nothing has yet told it not to will move during a
brownout, during a slow peripheral bring-up, and in the window between power-on
and the first sensor read. Those are exactly the moments when nobody is expecting
it to move.

**What it costs.** The robot cannot move for one control cycle after boot. Nobody
has ever wanted that cycle back.

---

## A live link is not enough

**Decision.** The interlock has two separate faults: `LinkLost`, and
`CommandStale` when no velocity setpoint has arrived within 300 ms.

**Why.** Checking only that the transport is up misses the failure that actually
happens: the controller behind it crashes, hangs, or gets stuck in a planner,
while the socket stays open and the carrier stays present. Every "is the link up"
check passes and the robot drives on the last setpoint it was given,
indefinitely.

A timeout on the *setpoints* catches that, and catches a partly wedged sender
too. It is the cheapest safety property in the whole system.

**What it costs.** A supervisor must send setpoints continuously, including
zeros, rather than only on change. That is the right protocol for a moving
machine anyway — an edge-triggered "stop" command that gets lost is a runaway.

**Choosing 300 ms.** Long enough for three missed 100 ms updates, short enough
that at 1 m/s the robot travels 30 cm before the interlock notices. Tune it
against your update rate and your stopping distance, not against a round number.

---

## Bumpers restrict a direction, they do not stop the robot

**Decision.** A pressed bumper zeroes only the linear component driving into it.
Reverse and rotation stay available.

**Why.** Two failure modes, and this sits between them. Ignoring the bumper means
driving into the obstacle. Treating it as a hard stop means the robot is now
immobile *while touching something*, and recovering it requires physically
lifting it — which people do, badly, in awkward places.

**What it costs.** A bumper alone will not bring the robot to a halt if the
operator keeps commanding reverse into a second obstacle. That is a real gap, and
the answer is the e-stop, not making the bumper more aggressive.

---

## The interlock is a class with tests, not an `if` in the control loop

**Decision.** Permission logic lives in one place with a truth table and 19 tests
against it.

**Why.** This is the interlock most likely to be disabled during bring-up. An
over-sensitive bumper is genuinely maddening to work around, and when the check
is three inline lines in a control loop, commenting it out is a one-line edit
nobody reviews, at 6pm, before a demo. It then stays commented out, because the
demo went fine.

Putting it behind a tested interface does not make that impossible. It makes it
loud: the diff deletes named test cases, and the reviewer sees which safety
property was given up.

**What it costs.** More ceremony than an `if`. Worth it for exactly one class of
logic and not much else.

---

## The half-track convention, stated twice and tested

**Decision.** `DriveGeometry::track_width_m` is the full track. The kinematics
halve it internally. There is a test asserting that 1 rad/s on a 0.40 m track
gives ±0.20 m/s at the wheels.

**Why.** "Wheel separation" is the most reliably confused constant in mobile
robotics. The body-to-wheel relation `v = ω·r` wants the half track, and passing
the full width gives exactly a factor of two in yaw.

What makes it nasty is that it hides. Forward and inverse kinematics normally
share the mistake, so the robot reports precisely the yaw rate it was asked for
while turning at half that rate in the real world. Open-loop it looks perfect.
It only surfaces when you close the loop against an external reference — a scan
match, a fiducial — by which point it is buried under whatever else is also
wrong.

**What it costs.** Nothing, beyond the discipline of writing the test.

---

## Saturation scales both wheels

**Decision.** When a commanded twist exceeds what the wheels can deliver, both
are scaled by one factor rather than clamped independently.

**Why.** Clamping independently changes the ratio between the wheels, and the
ratio *is* the turn radius. A robot asked to drive fast along a gentle arc would
progressively straighten out as it saturated — following a path nobody asked for,
at the moment it is going fastest. Scaling together means saturation costs speed
and never path.

**What it costs.** The robot goes slower than requested rather than turning
tighter than requested. That is the correct trade for anything following a plan.

---

## The exact arc, not the straight-line approximation

**Decision.** `Odometry::update` integrates the constant-curvature arc.

**Why.** The straight-line approximation — move `v·dt` along the current heading,
then rotate — is off by a term proportional to `ω·dt²`. Over a 10 ms step that is
tiny. The problem is that it is *biased*: always to the inside of the turn. It
does not average out, so on a robot that spends its life following curved aisles
it accumulates as a steady radial drift, and it looks exactly like a wheel radius
calibration error, which is what people spend a week chasing instead.

**What it costs.** A `sin` and a `cos` per update, and a small-angle branch to
avoid dividing by the yaw rate.

---

## Odometry integrates measured speeds, not commanded ones

**Decision.** `Robot::run_control` feeds `hal::Drivetrain::measured_speeds()` to
the odometry.

**Why.** Integrating the command gives an estimate that is beautifully smooth and
increasingly fictional. A stalled wheel, a slipping tyre, a driver in current
limit — all invisible. Worse, the failure is silent in exactly the situation
where you most want to notice: the robot is stuck, and its own telemetry says it
is making excellent progress.

**What it costs.** Encoder noise reaches the pose. That is the right problem to
have; filter it if it matters.

---

## Wrap-safe time, everywhere, tested across the wrap

**Decision.** All elapsed-time arithmetic goes through
`elapsed_since(then, now) = (uint32_t)(now - then)`. Deadlines and the command
timeout each have a test that exercises the 32-bit wrap.

**Why.** `uint32_t` milliseconds wrap after 49.7 days. The intuitive guard,
`if (now > then)`, is wrong: after the wrap it reports enormous intervals, so
every "has 500 ms elapsed" test fires continuously. Unsigned subtraction is
correct by construction as long as less than 2³² ms have genuinely passed.

The reason this needs a test rather than a code review is that it is unreachable
in practice — nobody leaves a bench robot running for seven weeks — so it ships,
and it fails in the field on a machine that has been up since installation.

**What it costs.** Nothing. It is less code than the wrong version.

---

## The watchdog is kicked once, at the end

**Decision.** `hal::Watchdog::kick()` is called from exactly one place: the last
statement of `Robot::step()`.

**Why.** A watchdog kicked from a timer interrupt only proves the chip is
executing interrupts. The control loop can be wedged in an infinite loop, blocked
on a peripheral that never responds, or stuck in a state machine, and the
interrupt keeps firing and keeps the robot alive — with its last commanded
velocity latched into the drivetrain. That is the failure the watchdog was bought
to prevent, and kicking it from a timer is how you buy it and not get it.

Kicking at the end of a completed iteration means the kick is evidence that the
loop actually ran.

**What it costs.** The timeout must accommodate the worst-case iteration, not the
typical one. 100 ms against a 10 ms period is ten cycles of slack, which is
generous; tighten it once you have measured your real jitter.

---

## The safety lane rejects; the others overwrite

**Decision.** `Priority::High` uses `OverflowPolicy::Reject` and latches a fault
on rejection. `Normal` and `Low` use `DropOldest`.

**Why.** These are different kinds of message. A velocity setpoint is *level*
data — the newest supersedes the last, and queueing them means the robot works
through a backlog of stale motion after any stall. A bumper edge is *event* data:
press and release are not interchangeable, and losing one leaves the system
believing a button is still held.

So the safety lane must not silently discard. If it ever fills, something has
gone wrong upstream and we cannot know what was lost, so the interlock treats it
as a hard fault requiring human intervention.

**What it costs.** A depth choice. Sixteen is comfortably more than the discrete
events that can plausibly land in one 10 ms period; if it overflows, the bug is
the flood, not the depth.

---

## Text on the wire

**Decision.** Telemetry and commands are line-oriented ASCII.

**Why.** A few hundred bytes at 20 Hz makes the bandwidth argument for a packed
binary struct irrelevant, while its costs are real: silent misinterpretation when
the two ends are built from different headers, and a wire you cannot inspect
without writing tooling first. Being able to `cat` the link, or type a command by
hand, is worth a great deal when something is wrong at 3am.

**What it costs.** Parsing, and `snprintf`, which is not free on a small part.
If you need the bandwidth, keep the text format available on a debug endpoint.

---

## The parser rejects rather than guesses

**Decision.** `app::parse` returns false for anything not exactly understood.
NaN, infinity, trailing characters, missing arguments, over-length lines — all
rejected.

**Why.** This is the robot's command path and the only place untrusted input
enters. A permissive parser is both an attack surface and a fault source: a
NaN reaching the kinematics propagates into the pose and never leaves, since
every subsequent comparison against it is false.

Over-length lines are discarded whole rather than truncated, because a truncated
line can still parse. `V 1.0 2.5` cut short becomes `V 1.0 2` — valid, and a
different command.

**What it costs.** A supervisor with a sloppy encoder gets silence instead of
approximate obedience. That is the correct direction to fail.

---

## The gauge compensates for load and refuses to creep upward

**Decision.** Open-circuit voltage is recovered as `V_terminal + I·R` before the
curve lookup, and the reported charge does not rise except on a large sustained
jump.

**Why.** Two separate problems, both of which make a gauge useless in the same
way — nobody believes it.

Terminal voltage under load is depressed by I·R, so reading the curve directly
makes the pack look empty every time the robot accelerates and full again
whenever it coasts. On a 20 A load through 20 mΩ that is 0.4 V, which on the
steep part of the curve is tens of percent.

And every noise source in the measurement is symmetric while the underlying
quantity only decreases. Without a monotonicity rule the display jitters upward
constantly.

**What it costs.** A genuine recharge is not reflected until it has persisted for
several samples. That is a few hundred milliseconds, against a process that takes
hours.

---

## No RTOS

**Decision.** One cooperative loop.

**Why.** For a fixed set of periodic activities with no blocking operations, an
RTOS adds stack sizing, priority assignment, and the possibility of inversion,
in exchange for preemption that nothing here needs. The single-context version
has timing you can determine by reading it.

**What it costs.** Every activity must be non-blocking and short — no `delay()`,
no synchronous flash writes, no busy-waiting on a peripheral. That constraint is
load-bearing, and it is the first thing to break when somebody adds a feature in
a hurry.

**When to change.** If you add something genuinely long-running — SLAM, a file
system, an encrypted link — move the control loop into its own task and keep the
command bus as the boundary between it and everything else. The decomposition
already anticipates that; the bus is the thread boundary.
