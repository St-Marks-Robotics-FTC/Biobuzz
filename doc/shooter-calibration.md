# Shooter calibration

How to get `subsys/physics.java` producing RPMs that actually score. Work top to bottom — each
phase assumes the ones above it are done. Skipping ahead means fitting constants against noise.

There are two outtakes and **they share nothing**. Outtake 1 (POLLEN) and outtake 2 (NECTAR) each
have their own hood angle, exit point, speed line and drag fit. Every phase below gets done twice,
once per outtake. Don't assume the second one inherits anything from the first.

Every constant lives in `Tunables` and is `@Configurable`, so you can edit all of it live from
Panels at [192.168.43.1:8001](http://192.168.43.1:8001) without redeploying. The whole procedure is
built around changing one number and shooting again.

## What you're solving for

Per outtake, six numbers. `pollen*` is outtake 1, `nectar*` is outtake 2.

| Constant | How you get it | Phase |
|:---|:---|:---|
| `*LaunchAngleDeg` | measure with an angle finder | 0 |
| `*LaunchHeightIn` | measure with a tape | 0 |
| `*LaunchOffsetIn` | measure with a tape | 0 |
| `*SpeedPerRpm` | fit from the measured speed line | 2 |
| `*SpeedIntercept` | fit from the measured speed line | 2 |
| `*DragCoefficient` | fit from long-range shots | 3 |

The first three are measurements. Do not fit them — if you let them float they'll absorb error that
belongs elsewhere and the model stops extrapolating.

### Why there's no wheel diameter or efficiency here

There used to be `flywheelDiameterIn` and `flywheelTransfer`. They were **perfectly degenerate** —
the code only ever used their product, so no amount of shooting could tell you which one was wrong.
They're now collapsed into one measured line:

```
muzzle speed (m/s) = SpeedPerRpm * RPM + SpeedIntercept
```

Two numbers, both identifiable, both measurable. The intercept is usually slightly negative: the
ball slips before it grips, so the line doesn't pass through the origin. If you force it through
zero you'll be biased low at low RPM and high at high RPM.

To seed the slope before you have data, `physics.seedSpeedPerRpm(wheelDiameterIn, transfer)` gives
the geometric guess. A 4 in wheel at 0.5 transfer is 0.00266, which is the current default.

---

## Phase 0 — bench measurements

Robot on tiles, battery in, nothing spinning. **Do this twice, once per outtake.**

1. **Hood angle.** Digital angle finder on the *launch axis* — the line from the wheel contact
   point through the exit — not on the chassis or a convenient flat face. → `*LaunchAngleDeg`
2. **Exit height.** Height of the ball's *centre* at the moment it leaves contact with the wheel,
   measured from the tiles. Load a ball and measure it in place. → `*LaunchHeightIn`
3. **Exit offset.** Horizontal distance from the Pinpoint's tracking centre to that exit point,
   positive if the exit is ahead of the odo centre. → `*LaunchOffsetIn`

The two outtakes almost certainly differ on all three. A 2 in difference in exit height is worth
about 40 RPM; a 3° difference in hood angle is worth far more.

## Phase 1 — flywheel control (do not skip)

The model's speed budget is about **±2.5% at 108 in and ±5% at 48 in**. If a flywheel can't hold
RPM tighter than that, no amount of physics tuning will help and every measurement from Phase 2
onward is noise.

1. Add both shooter motors to the config **with encoders**. Neither is in the hardware map yet —
   see the expansion hub table in the README.
2. Close the loop on velocity with `subsys/PIDF.java`, one controller per outtake with its own
   gains. Feedforward (`kF`) does most of the work; `kP` cleans up the rest. Integral usually isn't
   needed and will wind up during the spin-down after a shot.
3. Graph RPM against time in Panels through a 5-ball burst.

**Gate:** steady-state within ±1%, and recovery to within ±1% in under 0.75 s after each ball.
Shot-to-shot recovery is what actually kills scoring, not steady-state. Gate the trigger on
"at speed" rather than trusting a timer.

## Phase 2 — measure the speed line

This is the phase that makes everything else work, and it's the one you asked about. You need at
least **three RPM points per outtake**, spanning the range you'll actually use (roughly 1800–3000
RPM at the current defaults), then fit a straight line through them.

### Option A — chronograph (better)

An airsoft chronograph is about $30 and reads muzzle velocity directly. Fire five balls at each of
three or four RPM settings, average each, and least-squares fit speed against RPM. This decouples
the launcher completely from the ballistics, so a bad Cd can't corrupt your speed line.

If you can get one, do this. It turns Phase 3 into a check rather than a fit.

### Option B — shoot and solve (no extra hardware)

1. Park at a **measured 48 in** from the aperture centre. Tape-measure it; don't trust odometry
   yet. Close range keeps flight time short, so drag barely contributes and can't poison the fit.
2. Sweep RPM by hand until shots land in the middle of the opening, front to back. Five balls per
   setting. Record `RPM_centred`.
3. Read the muzzle speed the solver wanted at that range from telemetry (`Shot.ballSpeed`). That
   pair — `(RPM_centred, ballSpeed)` — is one point on your line.
4. Repeat at **72 in** and **100 in** for two more points.
5. Fit slope and intercept through the three points:

   ```
   SpeedPerRpm   = (v_far - v_near) / (rpm_far - rpm_near)
   SpeedIntercept = v_near - SpeedPerRpm * rpm_near
   ```

Sanity check: slope should land within about 20% of `seedSpeedPerRpm()` for your wheel. If it's
miles off, re-measure the compressed wheel diameter — a 4 in compliant wheel squashed 0.25 in
behaves like 3.75 in.

## Phase 3 — fit `*DragCoefficient` per ball

The speed line is now fixed. Don't touch it in this phase.

POLLEN and NECTAR need **separate** drag coefficients. NECTAR is a holed, pickleball-style shell;
POLLEN is smaller and smoother. They do not have the same Cd and there's no reason to expect them
to. This matters: at 96 in, sweeping Cd from 0.35 to 0.80 moves the answer 2517 → 2694 RPM, about
7% — larger than the whole ±2.8% error budget at that range.

1. Park at a **measured 100 in**. Let the solver command the RPM.
2. Shoot five.
   - Falling **short** → drag is underestimated → raise `*DragCoefficient`.
   - Going **long** → lower it.
3. Step in 0.05 increments until centred. Sane range is 0.4–0.7.

If you need something outside 0.3–0.9, the problem isn't drag. Go back to Phase 2 — most likely
the speed line was fit with a flywheel that hadn't recovered between shots.

## Phase 4 — verify across the triangle

Five balls at each of six ranges spanning your triangle, plus each vertex, **per outtake**. Record
hits and, for every miss, whether it was **long or short** — that's the diagnostic signal.

| Symptom | Cause | Fix |
|:---|:---|:---|
| Same-sign bias at every range | speed line intercept | Redo Phase 2 |
| Bias grows with range | `*DragCoefficient` | Redo Phase 3 |
| Bias grows *near* only | exit height or hood angle | Re-measure, Phase 0 |
| One outtake good, one bad | you copied constants between them | Redo Phase 0 for the bad one |
| Random scatter, no pattern | flywheel recovery or ball variation | Back to Phase 1 |
| Left/right misses | heading, not speed | Phase 5 |

**Accept at 4/5 or better at every vertex.** The opening is 20 in wide but only about 12 in tall in
the vertical, so nearly every genuine miss is a speed problem, not an aiming one.

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

1. One range, three balls, from a vertex you know, per outtake.
2. Adjust **`*SpeedIntercept` only** — it shifts every range by the same amount, which is what a
   fresh field actually does to you. Leave the slope alone.
3. Check at a fresh battery and again at a low one. If RPM sags under load, that's `kF` and a
   voltage-compensated feedforward, not the physics.

Don't re-derive drag at an event. If the far shots are off and the close ones are fine, you have a
flywheel recovery problem showing up as range-dependent error.

## Guardrails

- Always check `Shot.feasible` before firing. It's false when no speed reaches the aperture, when
  the ball would arrive rising more steeply than the 60° mouth plane (it physically cannot enter),
  and when the speed line hasn't been calibrated.
- **Don't shoot from inside ~45 in.** At 65° the ball is still rising when it arrives and clips the
  lower lip. At 24 in the solver correctly refuses.
- `Shot.openingFrac` is how square-on the entry is, 1.0 being dead down the mouth normal. Below
  about 0.5 you're grazing the lip; treat it as a "reposition" signal in TeleOp.

## Reference numbers

Model output at 65°, POLLEN, 12 in exit, `SpeedPerRpm` 0.00266, `SpeedIntercept` 0, Cd 0.5. Yours
will differ once calibrated — this is for sanity-checking the order of magnitude, not a target.

| Range | Muzzle speed | RPM | Entry angle | Flight |
|---:|---:|---:|---:|---:|
| 48 in | 5.58 m/s | 2097 | 12.8° | 0.54 s |
| 60 in | 5.80 m/s | 2182 | 32.9° | 0.66 s |
| 72 in | 6.11 m/s | 2299 | 43.1° | 0.76 s |
| 84 in | 6.45 m/s | 2426 | 49.0° | 0.85 s |
| 96 in | 6.80 m/s | 2558 | 52.9° | 0.93 s |
| 108 in | 7.16 m/s | 2690 | 55.6° | 1.00 s |

## Log

Fill this in and commit it. Re-measure after any hood, wheel, or intake change.

| Date | Outtake | Angle | Exit ht | Offset | SpeedPerRpm | Intercept | Cd | Notes |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|
| | | | | | | | | |
