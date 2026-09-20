# Shooter calibration

How to get `subsys/physics.java` producing RPMs that actually score. Work top to bottom — each
phase assumes the ones above it are done. Skipping ahead means fitting constants against noise.

Every constant below lives in `Tunables` and is `@Configurable`, so you can edit all of it live
from Panels at [192.168.43.1:8001](http://192.168.43.1:8001) without redeploying. Use that. The
whole procedure is built around changing one number and shooting again.

## What you're solving for

| Constant | How you get it | Phase |
|:---|:---|:---|
| `launchAngleDeg` | measure with an angle finder | 0 |
| `launchHeightIn` | measure with a tape | 0 |
| `launchOffsetIn` | measure with a tape | 0 |
| `flywheelDiameterIn` | measure **compressed** | 0 |
| `flywheelTransfer` | fit from close-range shots | 2 |
| `ballDragCoefficient` | fit from long-range shots | 3 |

The first four are measurements. Do not fit them — if you let them float they'll absorb error that
belongs elsewhere and the model stops extrapolating.

`flywheelTransfer` and `ballDragCoefficient` both push the commanded RPM the same direction, so
they can't be separated from a single distance. They separate by **range**: transfer is a flat
scale on RPM at every distance, drag grows with flight time. That's why you fit transfer close and
drag far, in that order, and never touch them at the same time.

---

## Phase 0 — bench measurements

Robot on tiles, battery in, nothing spinning.

1. **Hood angle.** Digital angle finder on the *launch axis* — the line from the wheel contact
   point through the exit — not on the chassis or a convenient flat face. Put the measured number
   in `launchAngleDeg`. If it isn't 65°, that's fine; the solver takes whatever you give it.
2. **Exit height.** Height of the ball's *centre* at the moment it leaves contact with the wheel,
   measured from the tiles. Load a ball and measure it in place. → `launchHeightIn`
3. **Exit offset.** Horizontal distance from the Pinpoint's tracking centre to that exit point,
   positive if the exit is ahead of the odo centre. → `launchOffsetIn`
4. **Compressed wheel diameter.** With a ball loaded, measure wheel axle to ball contact and
   double it. A 4 in compliant wheel squashed 0.25 in behaves like 3.75 in, which is a 7% RPM
   error if you use the nominal number. → `flywheelDiameterIn`

Record these in the table at the bottom. Re-measure any time the hood or wheel changes.

## Phase 1 — flywheel control (do not skip)

The model's speed budget is about **±2.5% at 108 in and ±5% at 48 in**. If the flywheel can't hold
RPM tighter than that, no amount of physics tuning will help and every measurement in Phase 2
onward is noise.

1. Add the shooter motor to the config with its encoder. It isn't in the hardware map yet — see
   the expansion hub table in the README.
2. Close the loop on velocity with `subsys/PIDF.java`. Feedforward (`kF`) does most of the work;
   `kP` cleans up the rest. Integral usually isn't needed and will wind up during the spin-down
   after a shot.
3. Graph RPM against time in Panels through a 5-ball burst.

**Gate:** steady-state within ±1%, and recovery to within ±1% in under 0.75 s after each ball.
Shot-to-shot recovery is what actually kills scoring, not steady-state. Don't move on until both
hold. Gate the trigger on "at speed" rather than trusting a timer.

## Phase 2 — fit `flywheelTransfer` at close range

This is the one constant that matters most, and the default 0.5 is a placeholder.

1. Park at a **measured 48 in** from the aperture centre. Tape-measure it; don't trust odometry
   yet. Aim the robot at the cell.
2. Read the ball speed the solver wants at that range from telemetry (`Shot.ballSpeed` — should be
   about 5.6 m/s at 65° with a 12 in exit).
3. Ignore the solver's RPM for now. **Sweep RPM by hand** until shots land in the middle of the
   opening, front to back. Five balls per setting. Call the result `RPM_centred`.
4. Compute:

   ```
   flywheelTransfer = ballSpeed / (pi * flywheelDiameterIn * 0.0254 * RPM_centred / 60)
   ```

   Put that in Panels. The solver's commanded RPM should now land on `RPM_centred`.

Expect roughly 0.4–0.6 for a single wheel against a hood, 0.8–0.9 for two opposed driven wheels.
Anything outside that means something upstream is wrong — usually the compressed wheel diameter,
or a ball that's slipping instead of gripping.

## Phase 3 — fit `ballDragCoefficient` at long range

Transfer is now fixed. Don't touch it again in this phase.

1. Park at a **measured 100 in**. Let the solver command the RPM.
2. Shoot five.
   - Falling **short** → drag is underestimated → raise `ballDragCoefficient`.
   - Going **long** → lower it.
3. Step it in 0.05 increments until centred. Sane range is 0.4–0.7; the default 0.5 is a
   holed-plastic-ball estimate, not a measurement.

If you need something outside 0.3–0.9 to make this work, the problem isn't drag. Go back to
Phase 2 — most likely the close-range fit was done with a flywheel that hadn't recovered.

## Phase 4 — verify across the triangle

Five balls at each of six ranges spanning your triangle, plus each vertex. Record hits and, for
every miss, whether it was **long or short** — that's the diagnostic signal.

| Symptom | Cause | Fix |
|:---|:---|:---|
| Same-sign bias at every range | `flywheelTransfer` | Redo Phase 2 |
| Bias grows with range | `ballDragCoefficient` | Redo Phase 3 |
| Bias grows *near* only | `launchHeightIn` or `launchAngleDeg` wrong | Re-measure, Phase 0 |
| Random scatter, no pattern | flywheel recovery or ball variation | Back to Phase 1 |
| Left/right misses | heading, not speed | Phase 5 |

**Accept at 4/5 or better at every vertex.** The opening is 20 in wide but only about 12 in tall
in the vertical, so nearly every genuine miss is a speed problem, not an aiming one.

## Phase 5 — target geometry

The `Cell` enum coordinates were derived from the manual's figures, not the field STEP file, so
confirm them before you trust auto-aim.

1. Drive to a known pose. Compare `Shot.headingRad` against the bearing the Limelight reports to
   the cell's AprilTag cluster.
2. A **constant offset** at every pose means the axis or a sign in the `Cell` enum is wrong — the
   hive arm runs audience-to-far, and red/blue are 25.5 in apart across it. An offset that
   *changes* with position means the Pedro pose is off, not the target.
3. Confirm you're reading which cell is up from the tag IDs: 30-33 and 34-37 are red, 38-41 and
   42-45 blue. **Getting this wrong moves the aim point 31 in** — a guaranteed miss, and the most
   likely cause of a shot that's perfectly calibrated and still wrong.

## Phase 6 — at the event

Field to field, tile compression and ball wear shift things a few percent. Budget five minutes in
your first practice match slot.

1. One range, three balls, from a vertex you know.
2. Adjust **`flywheelTransfer` only**. Everything else travelled with you.
3. Check at a fresh battery and again at a low one. If RPM sags under load, that's `kF` and a
   voltage-compensated feedforward, not the physics.

Don't re-derive drag at an event. If the far shots are off and the close ones are fine, you have a
flywheel recovery problem that's showing up as range-dependent error.

## Guardrails

- Always check `Shot.feasible` before firing. It's false when no speed can reach the aperture, and
  when the ball would arrive rising more steeply than the 60° mouth plane — it physically cannot
  enter.
- **Don't shoot from inside ~45 in.** At 65° the ball is still rising when it arrives and clips the
  lower lip. At 24 in the solver correctly refuses.
- `Shot.openingFrac` is how square-on the entry is, 1.0 being dead down the mouth normal. Below
  about 0.5 you're grazing the lip; treat it as a "reposition" signal in TeleOp.
- POLLEN and NECTAR need the same RPM within 0.6%, so one setting covers both. If they behave
  differently on the field, it's the intake or the feed, not the shot.

## Reference numbers

Model output at 65°, POLLEN, 12 in exit, 4 in wheel, `flywheelTransfer` 0.5. Yours will differ
once calibrated — this is for sanity-checking the order of magnitude, not a target.

| Range | Ball speed | RPM | Entry angle | Flight |
|---:|---:|---:|---:|---:|
| 48 in | 5.58 m/s | 2097 | 12.8° | 0.54 s |
| 60 in | 5.80 m/s | 2182 | 32.9° | 0.66 s |
| 72 in | 6.11 m/s | 2299 | 43.1° | 0.76 s |
| 84 in | 6.45 m/s | 2426 | 49.0° | 0.85 s |
| 96 in | 6.80 m/s | 2558 | 52.9° | 0.93 s |
| 108 in | 7.16 m/s | 2690 | 55.6° | 1.00 s |

## Log

Fill this in and commit it. Re-measure after any hood, wheel, or intake change.

| Date | Angle | Exit ht | Offset | Wheel dia | Transfer | Cd | Notes |
|:---|:---|:---|:---|:---|:---|:---|:---|
| | | | | | | | |
