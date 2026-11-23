# 9/2

Ran FusionTestOpMode on the partial INTO THE DEEP field.

## 1. Vision valid

MT1 sometimes has no data; all other localizers are normal.

## 2. Vision invalid / impact

Pinpoint drifts slightly; the other localizers recover once vision returns.

## 3. Vision valid / far from tags

**Problem**: when the robot is far from the tags, vision error is huge; it cannot correct EKF/UKF and instead drags them off.

***

## Fixes (implemented 9/3)

### Diagnosis of scenario 3

Far-field vision error is far larger than the `stdDev` reported by MT1, and the old implementation had two flaws:

1. **R upper clamp**: `mapStdToR` clamped R at `0.01 x R_MAX_SCALE` at `STD_HIGH`, so no matter how far the distance or how large stdDev grew, the vision measurement noise never kept growing, and the filter kept over-trusting far-field vision solves.
2. **No outlier rejection**: distorted vision solves were consumed directly as valid measurement updates, so a single bad observation could drag the pose off.

### Concrete fixes (AdaptiveEKF/UKF)

1. **Remove the R cap**: `mapStdToR` is now an unbounded linear map; past `std > STD_HIGH` it keeps scaling linearly with no ceiling.
2. **Quadratic distance scaling**: R on the position axes is additionally multiplied by `distFactor = (dist / DIST_REF_M)^2` (when `dist > DIST_REF_M`), reflecting that vision position error grows roughly quadratically with distance.
3. **Tag-count scaling**: R on the position axes is additionally multiplied by `tagFactor = TAG_REF / tags` (when tags `< TAG_REF`, capped by `TAG_SCALE_MAX`); fewer tags means a less reliable solve.
4. **Mahalanobis gating**: before each vision update, `gateVision()` computes `d^2 = innov^T S^-1 innov`, and the update is accepted only when `d^2 <= GATE_THRESHOLD^2`; the rest are discarded (predict continues as usual, relying on odometry until the next trusted observation).

See [Theory.md](./Theory.md) Sec. 5.3-5.5, [Theory5D.md](./Theory5D.md) Sec. 6.4, and the top of [Params.md](./Params.md), "Adaptive vision R and Mahalanobis gating".

### To verify

- Far from tags, the pose should no longer be dragged off by vision; after vision recovers it should only correct slowly.
- Scenario 1 (MT1 occasionally producing no data) is untouched for now; still need to separate occlusion vs. exposure vs. compute/frame-rate causes.
- <br />

***

# 9/3

Ran FusionTestOpMode on the partial INTO THE DEEP field.

## Accuracy test

Kept vision valid and hit the robot. Observed pinpoint drift and vision oscillation; the other localizers stayed accurate.

## Vision robustness test

Observed whether vision drags the filter while far from AprilTags. EKF/UKF oscillated hard with vision; **AdaptiveEKF/UKF accumulated very large R, large pose errors were rejected by gating, and localization stayed stable**.

## Convergence speed test

Blocked the tags, hit the robot by hand to force a large pose error, then briefly restored vision and cut it again.

- Pinpoint: completely dead.
- EKF/UKF: slow convergence, never fully re-localized before vision dropped again.
- **AdaptiveEKF/UKF: accumulated very large Q, converged extremely fast with accurate localization.**
- No significant difference between EKF and UKF of the same family.

## Next steps

Test on the full BIOBUZZ field to simulate the real match environment, log errors quantitatively, and verify localizer performance.

Test D3 and 5D localization on the ramped field.

***

# 9/10

Tested Fusion5D_D2TestOpmode (D2-mode 5D fusion) on the partial INTO THE DEEP field.

## Lift test

With the robot lifted (wheels off the ground):

- **AdaptiveEKF (D2) stops updating** - the pose barely changes; there is no valid motion/observation info while lifted.
- **5D localizers (EKF5D / UKF5D) keep updating** - the accelerometer control input still drives prediction, but **with large error**: the lifted segment has no external observation to correct it, so pure-inertial double integration drifts fast.

## Normal driving accuracy

When not lifted, driving normally:

- **5D localizers drift continuously**;
- **AEKF error is far below the 5D localizers** - vision/odometry inputs are identical; the only difference is that 5D additionally feeds in the accelerometer control input.

## Same-family comparison

Consistent with the 9/3 conclusion: **no clear performance difference observed between EKF5D and UKF5D**.

## Accelerometer measurements (AccelTest + Pinpoint)

| Item | Measured value |
| ---- | -------------- |
| BNO055 (ExpImu) reading refresh rate | about **0.7 s per sample (about 1.4 Hz)** |
| Reading noise range | +/-1 in/s^2 |
| Chassis max speed | about 70 in/s |
| Max accel from standstill | about 140 in/s^2 |
| Max decel | about -200 in/s^2 |

## Revised diagnosis: update rate (latency) dominates, not noise

- Accelerating to full speed takes about `70/140 = 0.5 s`, braking from full speed takes about `70/200 = 0.35 s`, **both shorter than one 0.7 s sample period** ->
  within one maneuver the accelerometer reports nearly a single "held value", i.e. **undersampling + zero-order-hold lag**; yet `predict()`
  integrates that held value times `dt` at 50 Hz every frame, so the lag error is injected repeatedly.
- Order-of-magnitude estimate: velocity bias is about `a_err dt / K` (`K` = odometry correction gain on velocity). Taking
  `a_err = 140 in/s^2`, `dt = 0.02 s`, `K ~ 0.09` -> steady-state velocity bias of about **30 in/s** (about 45% of max speed);
  if the decel segment still reports +140 (true value -200), then `a_err ~ 340` and the bias reaches **~70 in/s**.
  Pure zero-order-hold bound: `1/2 a t^2 = 1/2 x 140 x 0.7^2 ~ 34 in`, `delta-v = a t ~ 98 in/s`.
- **Noise is negligible**: with sigma = 1 in/s^2, the 5 s position random walk std is about 0.65 in, far below the systematic error above.
- So the earlier "noisy" hypothesis is revised to: **the update rate is too low (about 1.4 Hz), causing a 0.7 s-level systematic lag**.

### Root cause confirmed: driver poll interval (verified by measurement)

[ExpImu.java](../Accelerometer/ExpImu.java) calls at init
`imu.startAccelerationIntegration(new Position(), new Velocity(), 1000)`, where `1000` is the
**driver poll interval (ms)**, closely matching the measured ~1 s refresh rate (the BNO055 NDOF output rate itself is far higher).
It is now the Dashboard-tunable `ExpImu.INTEGRATION_POLL_MS`.

**Measured comparison (AccelTest)**:

| `INTEGRATION_POLL_MS` | Measured refresh rate | Lag | `int a dt` vs Pinpoint velocity |
| --------------------- | --------------------- | --- | ------------------------------- |
| 1000 (original) | about 1.4 Hz (0.7 s per sample) | 0.7 s level | clearly lagging / biased |
| **15** | **about 35 frames/s** | **nearly gone** | **waveforms nearly match** |

- At 15 ms the refresh rate rises to about 35 Hz and climbs no further; the bottleneck has moved to **the I2C cost of reading the whole BNO055 register block**,
  and lowering the parameter further brings no gain.
- Conclusion: the root cause of the 0.7 s refresh rate is indeed `msPollInterval = 1000`, **not BNO055 bandwidth**; it is a pure software-config
  issue, fixable through that parameter.

## Accelerometer static bias (measured)

- At rest, ax / ay show a **stable 1-5 in/s^2 bias**, with readings oscillating slightly around it.
- **After moving and stopping again, the bias changes, but stays constant while at rest.**

**Cause**: the NDOF linear acceleration output is `a_lin = a_meas - g_est` (measured acceleration minus the fused gravity-vector estimate);
the at-rest residual is **gravity-direction estimation error + the accelerometer's own bias**. A 1-5 in/s^2 level bias corresponds to an attitude error of
`delta = asin(a/g)` of about **0.15-0.74 deg** (`g = 386.09 in/s^2`), a normal magnitude for consumer-grade fusion.
Note that **yaw / magnetometer error cannot leak gravity**, so the problem is at the roll / pitch estimation level.

**Why the bias changes after motion but not at rest**: during motion the attitude is dominated by gyro integration, while true dynamic acceleration contaminates
the accelerometer tilt correction and the internal bias/calibration state is updated; after stopping, the fusion re-converges to a **new** steady-state error, so it
stays constant while at rest (if left resting longer, it may still slowly re-converge).

**Impact**:

- The bias is a DC term, growing as `1/2 b t^2` under double integration: `b = 3 in/s^2` -> **37.5 in** at 5 s, **150 in** at 10 s.
  And the bias direction/magnitude changes after every maneuver -> matches the "5D drifts continuously and irregularly" observation.
- At rest `|a|` is 1-5 in/s^2, far above the ZUPT threshold `0.5 in/s^2` -> **zero-velocity detection never fires**.

## Explaining the observations

| Observation | Explanation |
| ----------- | ----------- |
| 5D keeps updating while lifted but with large error | No external observations; prediction relies entirely on 0.7 s-lagged acceleration integration. |
| 5D drifts continuously while driving on the ground | Accel/decel finish within 0.35-0.5 s; the input lags and may even point the wrong way. |
| AEKF far better than 5D | AEKF uses no acceleration; velocity comes from Pinpoint (fast) + vision. |
| EKF5D ~= UKF5D | Same undersampled input path; the filter form cannot recover the missing bandwidth. |

## Next steps

- **Update rate: fixed** (`INTEGRATION_POLL_MS = 15` -> about 35 Hz, lag nearly gone). In
  `Fusion5D_D2TestOpmode`, set that value **before** constructing the localizers, re-run the 5D (D2/D3) vs AEKF comparison,
  and re-evaluate whether 5D is usable; the earlier "5D channel unusable at the current sample rate" conclusion is **on hold**.
- **Bias**: exploit the "bias stays constant at rest" property to estimate and subtract the
  ax/ay bias online at every stop (Pinpoint velocity about 0); switch the ZUPT criterion to **Pinpoint velocity**, no longer relying on the `|a|` threshold.
- Keep quantifying with `int a dt` vs Pinpoint velocity (already added in AccelTest; re-test cleared with **X**, should match at 15 ms).
- ~~Low-pass filtering~~ is demoted to a last resort: it only adds lag, the wrong direction.
- Quantitatively compare D2 / D3 and 5D localizers on the full field and the ramped course.

***

# 9/11 (re-test after calibration)

First finished manual BNO055 calibration with `AccelCalibration` (all four reaching `s3 g3 a3 m3`), then re-ran
[AccelTest.java](../../opmodes/AccelTest.java#L297-L303) to re-measure the accelerometer error.

## Observation: error still varies with pose, and varies over time at the same pose

Using the "is the error from gravity" criterion - if the error came only from the gravity-direction residual delta (fixed in the world frame), then:

| Quantity that should stay constant | Meaning |
| ---------------------------------- | ------- |
| `err_world = R(theta) err_body` | heading-independent, constant |
| `|err|` (= `|err_world|`) | independent of any attitude, constant |
| `err_z` | heading-independent |

Measured result (the "gravity error invariants" block at AccelTest lines 297-303):

- The x / y components of `err_world` **still vary with the vehicle pose**, showing none of the expected "constancy".
- The error measured **at the same pose at different times** also differs.

Both criteria are rejected, so the **error comes from more than gravity**.

## Verdict: magnetic interference

Compared against gravity-type error (world-frame constant, constant magnitude), the measured error also contains a **body-fixed, time-drifting** component.
After checking references, the main source is judged to be **magnetic interference from nearby wiring**:

- NDOF fusion relies on the magnetometer for heading/attitude; motors, ESCs, and high-current battery leads all generate varying board-level magnetic fields.
- A changed magnetic environment invalidates the magnetometer calibration (including the just-saved MAG calibration) -> the fused attitude and linear-acceleration baseline shift with it.
- This also explains "same pose, different result at different times" - the disturbance source is the current-driven magnetic environment, not geometry.

## Decision: stop using the BNO055

Motors, ESCs, and high-current leads cannot be removed on the field, and the wire routing is fixed by the structure, so **the magnetic interference cannot be eliminated**.
Calibration files only freeze one moment's magnetic environment and cannot cover the field that keeps changing while running. The acceleration input the 5D localizer relies on is not zero-mean noise, so **the BNO055 is dropped as the 5D acceleration source**.

## 5D localizer verification conclusion

- **Feasibility verified**: the EKF5D / UKF5D (D2, D3 wiring) code path is complete; the fusion framework of `predict` with Pinpoint / vision observations
  holds, runs normally, and reaches the "implementable" level.
- **Performance below expectations**: limited by the hardware above (untrustworthy accelerometer readings), **the 5D localizer was not observed to beat the 3D localizer**.

## Revisions to the existing plan

- ~~Estimate and subtract the ax/ay bias online using the "bias stays constant at rest" property~~: **dropped** - the bias contains time-varying magnetic interference and cannot be removed with a constant.
- ~~Pin `INTEGRATION_POLL_MS = 15` in `Fusion5D_D2TestOpmode` and re-evaluate 5D~~: the update-rate issue (0.7 s -> about 35 Hz)
  is indeed fixed, but the accelerometer baseline is still polluted by the magnetic field, so **5D stays out of the match plan**.
- The match localization plan stays centered on **3D localizers (AEKF/UKF: Pinpoint + vision)**.

