# Shooter calibration

What you have to do to make `subsys/physics.java` score. There are five things. Everything else in
this file is detail on those five.

1. **Measure the hood angle and the exit height.** Per outtake. Angle finder and a tape.
2. **Measure the rpm → muzzle speed line.** Per outtake. Fit a straight line, put in slope and
   intercept. This is the one that matters most.
3. **Get one drag coefficient per ball.** One number each, from one shot at long range.
4. **Make sure the flywheel is heavy enough and recovers between balls.** Physical, not software.
5. **Verify the target geometry** — the `Cell` coordinates and which cell is up.

Nothing is derived from wheel diameter, gear ratio or slip efficiency. Those were guesses and
they're gone; the measured line replaces all of them.

Two outtakes, and **they share nothing**. Outtake 1 (POLLEN) and outtake 2 (NECTAR) have their own
hood, wheel, angle, exit point, speed line and drag fit. Steps 1–3 get done twice.

Every constant lives in `Tunables` and is `@Configurable`, so you edit all of it live from Panels
at [192.168.43.1:8001](http://192.168.43.1:8001) with no redeploy.

## The constants

Per outtake, six numbers. `pollen*` is outtake 1, `nectar*` is outtake 2.

| Constant | Where it comes from | Step |
|:---|:---|:---|
| `*LaunchAngleDeg` | angle finder | 1 |
| `*LaunchHeightIn` | tape measure | 1 |
| `*LaunchOffsetIn` | tape measure, 0 is fine to start | 1 |
| `*SpeedPerRpm` | slope of your measured line | 2 |
| `*SpeedIntercept` | intercept of your measured line | 2 |
| `*DragCoefficient` | one long shot | 3 |

`SpeedPerRpm` and `SpeedIntercept` start at **zero**, and while they are zero the solver returns
`feasible = false` and an RPM of NaN. That is deliberate — it refuses to invent a number before you
have measured one. It will still tell you the muzzle speed it wants, which is what you need in
order to measure.

---

## Step 1 — hood angle and exit height

Robot on tiles, battery in, nothing spinning. Twice, once per outtake.

1. **Hood angle.** Digital angle finder on the *launch axis* — the line from the wheel contact
   point through the exit — not on the chassis or a convenient flat face. → `*LaunchAngleDeg`
2. **Exit height.** Height of the ball's *centre* as it leaves contact with the wheel, from the
   tiles. Load a ball and measure it in place. → `*LaunchHeightIn`
3. **Exit offset.** Horizontal distance from the Pinpoint's tracking centre to that exit point,
   positive if the exit is ahead. → `*LaunchOffsetIn`. Leave it at 0 until the rest works.

These are measurements, not fits. If you let them float they absorb error that belongs to the
speed line and the model stops extrapolating. The two outtakes will differ on all three — a 2 in
difference in exit height is worth about 40 RPM, and 3° of hood angle is worth much more.

## Step 2 — the rpm → speed line

Three or four RPM points per outtake, spanning what you'll actually use, then a least-squares line.

**Use a phone at 240 fps.** Don't buy an airsoft chronograph — the sensor aperture is about 1.5 in
and POLLEN is 2.8 in, so the ball will not fit through one.

1. Tape a ruler horizontally at the exit height, marked at 0 and 0.5 m downrange. Phone on a
   tripod square to the flight plane, 2–3 m back, 240 fps.
2. Five balls at each of three or four RPM settings.
3. Count frames `n` between the ball passing the two marks. `t = n / 240`.
4. Convert with the solver, which inverts the real trajectory:

   ```java
   double v0 = physics.muzzleSpeedFromVideo(0.5, n / 240.0, physics.Outtake.POLLEN);
   ```

5. Least-squares fit `v0` against RPM. Slope → `*SpeedPerRpm`, intercept → `*SpeedIntercept`.

**Do not compute this by hand as span ÷ time.** At a 65° hood that gives you the *horizontal*
speed, which is only `cos(65°)` = 0.42 of the muzzle speed, and gravity has already bled off a
chunk of it by the far mark. A 5.80 m/s shot reads as 2.39 m/s that way — 59% low. Correcting for
drag alone barely helps (2.42 m/s). `muzzleSpeedFromVideo` handles the launch angle, gravity along
the path and drag together, and round-trips exactly.

Cd does not have to be right yet: across the whole plausible range of 0.3 to 0.8 the recovered
speed moves only about 1%, because over half a metre drag has barely acted. So measure the line
first and fit Cd afterwards — there's no circularity.

The intercept is not decoration. The ball slips before it grips, so the line does not pass through
the origin; forcing it through zero biases you low at low RPM and high at high RPM.

Measuring the launcher on its own like this is what makes step 3 a clean one-parameter fit. If you
instead derive the line from where balls land, you are asking three noisy shots to determine slope,
intercept *and* drag at once, which is badly conditioned and tends to converge on nonsense.

## Step 3 — drag coefficient, per ball

POLLEN and NECTAR need **separate** coefficients. NECTAR is a holed, pickleball-style shell; POLLEN
is smaller and smoother. At 96 in, Cd 0.35 versus 0.80 is 2517 versus 2694 RPM — about 7%, wider
than the entire error budget at that range. A shared Cd guarantees one of the two is wrong.

1. Park at a **measured 100 in**. Let the solver command the RPM.
2. Shoot five. Short → raise `*DragCoefficient`. Long → lower it.
3. Step by 0.05 until centred. Sane range is 0.4–0.7.

Outside 0.3–0.9 means the problem isn't drag — go back and check the speed line was measured with a
flywheel that had recovered between shots.

## Step 4 — flywheel mass and recovery

This is physical and no amount of tuning substitutes for it.

Flywheel mass does **not** change the rpm → speed line. That line is measured, and it maps the
actual RPM at the instant of firing to the speed that comes out; the wheel's inertia is already
baked into it. Mass matters for exactly one reason: whether the actual RPM at firing equals the
RPM you commanded.

A ball leaving at 5.8 m/s carries about 0.44 J out of the wheel's stored energy, and speed drops as
`sqrt(1 - E_ball / E_wheel)`. A 100 g solid wheel at 4 in stores roughly 3.5 J at 2200 RPM, so one
ball knocks about 6–7% off it. Fire the next ball into that dip and the line faithfully gives you a
slow shot.

**So gate the trigger on measured RPM, not on a timer.** Do that and mass stops being an accuracy
problem and becomes a cycle-time one: recovery costs roughly `0.44 J / surplus motor power`, which
is tens of milliseconds with any reasonable headroom. Only if that wait is longer than the cycle
time you want does adding rim mass or gearing up become worth it — and measure it before deciding.

What actually needs headroom is the motor: run the flywheel well below its free speed so there is
torque left to recover with. A wheel spinning near free speed recovers slowly no matter how light.

Close the loop on velocity with `subsys/PIDF.java`, one controller per outtake with its own gains.
`kF` does most of the work, `kP` cleans up, integral usually just winds up during spin-down.

**Gate:** ±1% steady state, and back within ±1% before the next ball is fed. Graph it in Panels
through a 5-ball burst and read the recovery time off the plot — that number sets your cycle.

## Step 5 — target geometry

The `Cell` coordinates came from the manual's figures, not the field STEP file.

1. Drive to a known pose and compare `Shot.headingRad` against the bearing the Limelight reports to
   that cell's AprilTag cluster.
2. A **constant** offset at every pose means an axis or sign in `Cell` is wrong — the hive arm runs
   audience-to-far and red/blue are 25.5 in apart across it. An offset that **changes** with
   position means the Pedro pose is off, not the target.
3. Read which cell is up from the tag IDs: 30-33 and 34-37 red, 38-41 and 42-45 blue. **Getting
   this wrong moves the aim point 31 in** and is the most likely way to be perfectly calibrated and
   still miss everything.

---

## Where to shoot from

Put the three auto poses on an **arc at constant range**, spread in bearing. Skinny in range, wide
in bearing.

**Required speed is not monotonic in range — it bottoms out.** At the bottom `d(speed)/d(range)` is
zero, so being a few inches out of position costs nothing. At 60° the bottom is 54 in, and the
required RPM barely moves between 50 in and 58 in.

| Hood angle | Flat spot | Speed there | Entry quality | One RPM covers |
|---:|---:|---:|---:|:---|
| 58° | 58 in | 5.95 m/s | 0.87 | 50–76 in |
| **60°** | **54 in** | **5.82 m/s** | **0.88** | **47–70 in** |
| 62° | 50 in | 5.70 m/s | 0.88 | 44–65 in |
| 65° | 44 in | 5.55 m/s | 0.87 | 39–57 in |
| 70° | 34 in | 5.34 m/s | 0.87 | 31–43 in |

Speed tolerance at the flat spot is about 5.4% for *every* angle, so the angle doesn't pick a better
or worse spot — only where it lands and how far you can stray on one RPM. Shallower buys a wider
band. Bearing is nearly free: the opening is 20 in wide and only about 12 in tall, and you fix
bearing by turning.

**Recommended: 60° hood, three vertices on a 54 in arc, spread about ±25° in bearing.** One RPM
covers 47–70 in, so all three score off a single flywheel setting even if a vertex ends up 10 in
out of place.

Confirm the robot can actually reach the arc before committing the hood angle — the hive frame base
is 49.46 × 38.95 in and stops a bumper before the aperture does. The hood is not adjustable once
built.

## Diagnosing misses

Five balls at each of six ranges across your triangle, per outtake. For every miss record whether
it was **long or short** — that's the signal.

| Symptom | Cause | Fix |
|:---|:---|:---|
| Same-sign bias at every range | `*SpeedIntercept` | Redo step 2 |
| Bias grows with range | `*DragCoefficient` | Redo step 3 |
| Bias grows *near* only | exit height or hood angle | Re-measure, step 1 |
| One outtake good, one bad | constants copied between them | Redo step 1 for the bad one |
| Random scatter, no pattern | flywheel recovery | Step 4 |
| Left/right misses | heading, not speed | Step 5 |

Accept at 4/5 or better at every vertex. The opening is 20 in wide and about 12 in tall, so almost
every genuine miss is a speed problem, not an aiming one.

## Guardrails

- Check `Shot.feasible` before firing. It's false when the speed line isn't measured, when no speed
  reaches the aperture, and when the ball would arrive rising more steeply than the 60° mouth plane,
  which means it physically cannot enter.
- `Shot.openingFrac` is how square-on the entry is, 1.0 being straight down the mouth normal. Below
  about 0.5 you're grazing the lip — treat it as a reposition signal in TeleOp.
- At the event, adjust **`*SpeedIntercept` only**. It shifts every range by the same amount, which
  is what a fresh field actually does to you. Leave the slope and drag alone.

## Log

| Date | Outtake | Angle | Exit ht | Offset | SpeedPerRpm | Intercept | Cd | Notes |
|:---|:---|:---|:---|:---|:---|:---|:---|:---|
| | | | | | | | | |
