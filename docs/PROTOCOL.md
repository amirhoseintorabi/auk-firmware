# Link protocol

Line-oriented ASCII in both directions. Lines end with `\n`; a leading `\r` is
tolerated so either line ending works.

The format is text rather than a packed struct on purpose — see
[DESIGN-NOTES.md](DESIGN-NOTES.md#text-on-the-wire). It means you can drive the
robot from a serial terminal, and read its telemetry with `cat`.

## Host to robot

| Line | Meaning | Lane |
| --- | --- | --- |
| `V <linear> <angular>` | Body velocity: m/s forward, rad/s counter-clockwise | Normal |
| `L <0\|1>` | Work lamp off / on | Low |
| `E <0\|1>` | Assert / clear the software emergency stop | High |
| `C` | Clear latched faults, where the cause has gone | Low |

```
V 0.5 0.0      drive forward at 0.5 m/s
V 0.0 0.8      spin counter-clockwise at 0.8 rad/s
V 0.3 -0.4     forward and to the right
V 0 0          stop
L 1            lamp on
E 1            software e-stop
C              clear faults
```

### Setpoints must repeat

**`V` must be sent continuously, including zeros — not only on change.**

The robot cuts motion if no `V` arrives within 300 ms
(`safety::Interlock::kCommandTimeoutMs`). This is not a nuisance to work around:
it is what stops the robot when the sender crashes while its socket stays open.
An edge-triggered protocol where "stop" is a single message has a runaway
whenever that one message is lost.

10 Hz is a sensible rate. It gives three chances to make each 300 ms window.

### What gets rejected

The parser accepts only what it fully understands. Rejected input moves nothing
and produces no reply:

- `nan`, `inf`, or any magnitude above 10⁶
- trailing characters: `V 1.0 2.0abc`
- missing arguments: `V 1.0`
- unknown verbs: `VELOCITY 1 1`
- lines longer than 127 characters, which are discarded whole rather than
  truncated — a truncated line can still parse into a different valid command

`E` is posted to the high-priority lane, so it is never dropped in favour of a
newer message and is dispatched before any velocity command in the same cycle.

`C` clears operator-clearable faults. It deliberately cannot clear a physical
e-stop: that is released by the device, and a software override would make the
button advisory.

## Robot to host

One line per telemetry period, 20 Hz by default:

```
S <ms> <seq> x=<m> y=<m> yaw=<rad> v=<mps> w=<rps> cv=<mps> cw=<rps> soc=<0..1> volt=<v> lamp=<0|1> faults=<hex> drop=<n>
```

```
S 12100 242 x=2.179 y=0.557 yaw=1.5360 v=0.000 w=0.000 cv=0.000 cw=0.000 soc=0.020 volt=21.00 lamp=0 faults=0x0020 drop=0
```

| Field | Meaning |
| --- | --- |
| `<ms>` | Robot uptime in milliseconds. Wraps every 49.7 days |
| `<seq>` | Increments per frame. **Gaps mean frames were dropped** |
| `x`, `y`, `yaw` | Dead-reckoned pose, metres and radians. `yaw` is wrapped to (−π, π] |
| `v`, `w` | *Measured* body velocity, from encoders |
| `cv`, `cw` | *Commanded* body velocity, after interlock and slew limiting |
| `soc` | State of charge, 0 to 1 |
| `volt` | Open-circuit pack voltage, IR-compensated |
| `lamp` | Lamp state |
| `faults` | Bit field, hex — see below |
| `drop` | Commands dropped from the overwriting lanes since boot |

Telemetry is best effort. If the link cannot accept a frame it is dropped rather
than retried — the next one is 50 ms away, and blocking the control loop to
deliver a stale snapshot would be a poor trade. `<seq>` is how you detect it.

`v`/`w` against `cv`/`cw` is the most useful diagnostic pair on the wire. They
should track within the drivetrain's response time; a persistent gap means a
stalled wheel, a slipping tyre, or a driver in current limit.

### Fault bits

| Bit | Value | Meaning |
| --- | --- | --- |
| 0 | `0x0001` | Emergency stop asserted, from any source |
| 1 | `0x0002` | Front bumper pressed |
| 2 | `0x0004` | Rear bumper pressed |
| 3 | `0x0008` | Link down |
| 4 | `0x0010` | No velocity setpoint within the timeout |
| 5 | `0x0020` | Battery critical |
| 6 | `0x0040` | A safety-lane command was dropped |

All active faults are reported at once, not just the first.

Bits 1 and 2 restrict a direction rather than stopping the robot: with only a
bumper bit set, reverse and rotation remain available. Any other bit means motion
is fully blocked.

Bit 6 is the serious one. It means a discrete safety event — an e-stop press, a
bumper edge — was thrown away because the high-priority queue was full. Since we
cannot know what was lost, the robot stays down until it is power-cycled.

### `drop` is not necessarily a fault

The `Normal` lane holds one element and overwrites, so `drop` rises whenever two
velocity commands arrive inside one 10 ms control period. That is the intended
behaviour — only the newest setpoint matters — and at a 100 Hz command rate you
should expect it to climb steadily.

A sharp rise with no corresponding increase in command rate means housekeeping
commands are not being serviced. Bit 6, not this counter, is what tells you a
safety event was lost.
