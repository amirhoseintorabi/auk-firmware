# AUK

A small, complete firmware architecture for a differential-drive mobile robot —
written so that the entire control stack builds and runs on your workstation,
with no hardware and no RTOS.

AUK is a fictional robot. The firmware is real: command bus, cooperative
scheduler, kinematics, dead reckoning, safety interlock, battery gauge,
telemetry link. It exists to show how these pieces fit together, and to be a
reasonable starting point if you are building the same kind of thing.

```console
$ cmake -B build && cmake --build build && ./build/auk-sim-demo

AUK firmware -- simulated run
                             columns: ground truth / odometry estimate
------------------------------------------------------------------------------------
idle at power-on             true( 0.000,  0.000,   0.000)  odom( 0.000,  0.000,   0.000)  torque=off faults=command-stale
after 3 s forward            true( 1.933,  0.000,   0.000)  odom( 1.936,  0.000,   0.000)  torque=on  faults=none
after 2 s rotating           true( 2.160,  0.008,   1.405)  odom( 2.160,  0.008,   1.408)  torque=on  faults=none
front bumper, driving fwd    true( 2.160,  0.008,   1.536)  odom( 2.160,  0.008,   1.536)  torque=on  faults=front-bumper
reversing off the bumper     true( 2.150, -0.259,   1.536)  odom( 2.150, -0.254,   1.536)  torque=on  faults=front-bumper
1 s after link loss          true( 2.149, -0.295,   1.536)  odom( 2.149, -0.289,   1.536)  torque=off faults=link-lost
e-stop asserted              true( 2.149, -0.295,   1.536)  odom( 2.149, -0.289,   1.536)  torque=off faults=e-stop
2 s after e-stop release     true( 2.177,  0.506,   1.536)  odom( 2.177,  0.514,   1.536)  torque=on  faults=none
battery critical             true( 2.178,  0.551,   1.536)  odom( 2.179,  0.557,   1.536)  torque=off faults=battery-critical
```

That run is the real firmware. The only thing swapped out is the hardware layer.

## Why it is built this way

Small robot firmware tends to fail in a narrow set of ways, and most of them are
structural rather than clever. The design here is organised around closing those
off:

| Failure | What stops it |
| --- | --- |
| Six modules can each move the robot, and nobody knows which one did | Producers post commands; exactly one function acts on them |
| A safety check gets commented out during bring-up and never comes back | The interlock is one class with a truth table and 19 tests on it |
| The robot drives on for metres after the controller behind the link dies | A live link is not enough — setpoints must keep arriving |
| Yaw rate is out by exactly 2× and nobody notices | Forward and inverse kinematics derive from one geometry, and a round-trip test sweeps both axes |
| Everything works until 49.7 days of uptime | Every time comparison goes through wrap-safe arithmetic, and is tested across the wrap |
| The watchdog is kicked from a timer, so a wedged control loop keeps driving | It is kicked once, at the end of a completed iteration |
| A dropped safety event is invisible | The safety queue rejects rather than overwrites, and a rejection latches a fault |

[`docs/DESIGN-NOTES.md`](docs/DESIGN-NOTES.md) goes through each of these
properly, including the ones where the reasoning is not obvious.

## Layout

```
include/auk/
  core/       ring buffer, command bus, wrap-safe deadlines
  control/    differential-drive kinematics, odometry, slew limiting
  power/      battery gauge
  safety/     the interlock
  hal/        the hardware boundary -- pure interfaces, no platform headers
  app/        the robot loop and the telemetry codec
sim/          a simulated plant and a narrated demo
tests/        unit and end-to-end tests
```

`include/` and `src/` have no platform dependency at all: no allocation after
startup, no exceptions, no RTTI, no `<iostream>`. That is what lets the same code
run under the simulator and on a microcontroller.

## Building

Needs CMake 3.16 and a C++17 compiler. Nothing else — the test harness is 60
lines in `tests/check.hpp` rather than a dependency to fetch.

```sh
cmake -B build
cmake --build build
ctest --test-dir build --output-on-failure
./build/auk-sim-demo
```

Or run everything CI runs — gcc, clang, sanitizers, the Cortex-M4
cross-compile, clang-format and clang-tidy — in one go:

```sh
tools/check-all.sh
```

Missing tools are skipped rather than failing, so it is useful with whatever you
have installed. See [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md) for how to get
the full set, including without root.

The library builds under `-Wall -Wextra -Wpedantic -Wshadow -Wconversion
-Wsign-conversion -Wdouble-promotion -Wold-style-cast -Werror`. Warnings are part
of the build contract, not decoration.

### Footprint

`include/` and `src/` cross-compile for Cortex-M4 with `-fno-exceptions
-fno-rtti`, which CI checks on every push. At `MinSizeRel` the whole portable
library — command bus, kinematics, odometry, interlock, gauge, telemetry codec,
control loop — is:

```
   text    data     bss     dec  filename
   3867       0       0    3867  libauk.a (TOTALS)
```

No heap after startup, and all state is either a member or a stack local.

## Reading it

If you have ten minutes, read these three in order:

1. [`include/auk/safety/interlock.hpp`](include/auk/safety/interlock.hpp) — what
   the robot is allowed to do, and why it defaults to "nothing".
2. [`src/app/robot.cpp`](src/app/robot.cpp) — the whole control loop, in one
   file, in execution order.
3. [`tests/test_safety.cpp`](tests/test_safety.cpp) — every safety behaviour
   stated as a claim you can run.

## Documentation

| | |
| --- | --- |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | How the pieces fit, the execution model, and the data flow through one cycle |
| [`docs/DESIGN-NOTES.md`](docs/DESIGN-NOTES.md) | Why each decision went the way it did, and what it costs |
| [`docs/PORTING.md`](docs/PORTING.md) | Bringing it up on real hardware: what to implement, in what order, and how to know it works |
| [`docs/DEVELOPMENT.md`](docs/DEVELOPMENT.md) | Running the checks locally, the toolchain, and the conventions |
| [`docs/PROTOCOL.md`](docs/PROTOCOL.md) | The link protocol, both directions |

## Scope

This is a reference architecture, not a product. It has never driven a physical
robot, and it is not certified for anything.

If you put it on a machine that can hurt someone, the interlock in here is a
software convenience layer, not a safety system. A real one needs a hardware
e-stop circuit that removes motor power without asking any software for
permission, and if you need a rating, a design that has been assessed against
ISO 13849 or IEC 62061. Nothing in this repository substitutes for that.

## Licence

MIT — see [LICENSE](LICENSE). No third-party code is vendored; everything here
was written for this repository.
