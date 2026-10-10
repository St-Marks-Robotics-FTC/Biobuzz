# Fusion Tuning

`Fusion.java` combines odometry and Limelight vision into one pose. Odometry is smooth but drifts. Vision doesn't drift but is noisy and delayed. Fusion keeps an offset (how far odometry has drifted) and corrects it with every vision frame, trusting each source based on how uncertain it is.

All values below are in `Tunables.java` and can be changed live in [Panels](http://192.168.43.1:8001). Once something works, put the number in `Tunables.java` with the date, like the other tuned values.

## before you start

- odometry has to be tuned first. Fusion can't fix bad odometry, only drift.
  - must switch to `pedro-tuner` branch for Pedro tuning
- use the `VisionTuner` OpMode. It starts odometry at (72, 72) facing 90°, so put the robot in the middle of the field facing towards scoring. If you don't, the first frames get rejected until fusion catches up (a couple of seconds).
- `minTagCount` is 2, so the robot needs to see at least 2 tags or vision does nothing.

## telemetry

| name                       | meaning                                                        |
|----------------------------|----------------------------------------------------------------|
| `last botpose`             | raw vision position                                            |
| `odo pos`                  | raw odometry position                                          |
| `fused pos`                | final answer (odometry + offset)                               |
| `fusion offset x/y`        | how far odometry has drifted, in inches                        |
| `fusion std x/y`           | how unsure fusion is about its pose, in inches                 |
| `last vision std x/y`      | how unsure fusion thought the last vision frame was, in inches |
| `fusion accepted/rejected` | frame counts                                                   |
| `fusion last frame`        | whether the last frame was used, and if not, why               |

"std" is standard deviation. Read it as "usually off by about this much".

## tuning order

Do these in order. Each step depends on the ones before it.

### 1. vision noise: `fusionVisionBaseStd`, `fusionVisionRefDistance`

How noisy vision is when standing still at a known distance.

1. park the robot 48 in from the tags (or any distance, then set `fusionVisionRefDistance` to it). Don't move.
2. watch `last botpose` for about 10 seconds. Write down the lowest and highest x and y you see.
3. std ≈ (highest - lowest) / 4. Use the bigger of x and y.
4. vision math assumes 1 tag, but you're seeing 2. Multiply by 1.4 (√2).
5. set `fusionVisionBaseStd` to that.

Example: x bounces between 70.2 and 72.2. (72.2 - 70.2) / 4 = 0.5. 0.5 × 1.4 = 0.7.

### 2. check distance

Vision error grows with distance squared, so twice as far = 4x the error. Check that this is roughly true.

1. park far from the tags (about 100 in).
2. measure the spread like in step 1.
3. compare with `last vision std x/y`.
   - telemetry much smaller than what you measured: raise `fusionVisionBaseStd`
   - telemetry much bigger: lower it

If close and far can't both be right, favor the distance you shoot from most.

### 3. latency: `fusionExtraLatencyMs`

Vision is delayed. Fusion uses the delay the Limelight reports, but that might be a little off.

1. drive slowly back and forth in a straight line while seeing tags. Keep the speed steady.
2. watch `fusion offset x/y`. It should stay about the same.
3. if the offset moves back and forth with the robot, the delay is wrong.
   - offset gets pulled backwards as you drive (vision is behind the robot): raise it by 10
   - offset gets pulled forwards: lower it by 10
4. repeat until the offset stops following the robot.

### 4. motion: `fusionTimingErrorMs`, `fusionTurnStd`

How much less vision is trusted while moving. A moving or turning robot gives blurry, badly timed frames.

- drive and turn hard while seeing tags, then stop.
- `fused pos` jumps around while turning: raise `fusionTurnStd`
- `fused pos` jumps around while driving straight: raise `fusionTimingErrorMs`
- vision barely corrects anything while moving and the pose looks clean: you can lower them

### 5. odometry drift: `fusionOdoVariancePerInch`

How fast odometry gets worse as you drive.

1. let fusion settle while still and seeing tags (`fusion std` under 1).
2. cover the camera.
3. drive around for a while. Roughly track how far: a lap of the field is about 400 in.
4. come back, stop, uncover the camera. Write down how far `fused pos` jumps.
5. value = jump² / distance driven

Example: jumps 3 in after 500 in. 3 × 3 / 500 = 0.018.

Do it 2 or 3 times and average.

### 6. start: `fusionStartStd`

How wrong the starting pose usually is (the pose auto hands to TeleOp). If TeleOp usually starts within 3 in of the real spot, use 3. Bigger means the first vision frame snaps harder. The default of 6 is fine unless auto handoff is really good or really bad.

### 7. rejection: `fusionGateSigmas`, `fusionRejectsBeforeDoubt`

Fusion throws out frames that are way off from where it thinks it is.

- sit still with a clear view of the tags. Rejected should barely go up.
  - it keeps going up with "too far from current estimate": raise `fusionGateSigmas` to 4
- shove the robot about a foot while it's stopped. `fused pos` should fix itself in about 2 seconds.
  - takes too long: lower `fusionRejectsBeforeDoubt`
  - pose randomly jumps for no reason during normal driving: raise it

### 8. floor: `fusionMinVisionStd`

Fusion never trusts a single frame more than this. Leave it at 0.5. If `fused pos` jitters while standing still with tags in view, raise it.

## problems

| problem                                              | fix                                                                                        |
|------------------------------------------------------|--------------------------------------------------------------------------------------------|
| fused pose jitters while still                       | raise `fusionVisionBaseStd` or `fusionMinVisionStd`                                        |
| fused pose lags or overshoots when driving           | step 3 (latency)                                                                           |
| fused pose jumps when turning                        | raise `fusionTurnStd`                                                                      |
| vision never seems to correct anything               | check `fusion last frame`, check tag count is at least 2, lower `fusionVisionBaseStd`      |
| all frames say "too far from current estimate"       | robot isn't where odometry thinks. Wait for it to recover, or raise `fusionGateSigmas`     |
| all frames say "picture older than odometry history" | something is wrong with timestamps. Check `staleness` in telemetry, it should be under 100 |
| big jump after driving with no tags                  | normal if odometry drifted. If it's every time, redo step 5                                |
