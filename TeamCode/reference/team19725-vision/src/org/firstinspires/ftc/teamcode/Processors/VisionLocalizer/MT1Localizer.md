# MT1Localizer Design and Interface Description

This document describes the responsibilities and planned functionality of [MT1Localizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/MT1Localizer.java), including:

1. Obtaining the MegaTag1 botpose (raw vision pose) and quality metrics from the Limelight 3A;
2. Using a **single horizontal map** to solve for both the **HIVE tilt angle** and the **robot true pose**, following the derivation in [OneMap\_MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md);
3. Exposing a unified interface to upper layers (EKF / state machine).

> Note: Sections 1-3 are all implemented in the `.java` file. HIVE estimation is a **per-frame observation** with no time filtering
> and no saved history (hysteresis/hold is owned by the upper-layer RobotPosition).

***

## 1. Position in the Overall Localization Architecture

```
Limelight3A ──► MT1Localizer ──► AdaptiveEKFLocalizer ──► RobotPosition ──► state machine / chassis control
  (vision)      botpose fetch       EKF fusion                singleton external read
                hive estimation     (adaptive Q / R + gating)
```

- MT1Localizer implements the `Localizer` interface (see [Localizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RoadRunner/Localizer.java)),
  but it **only provides absolute pose observations, not velocity**; `update()` always returns zero velocity.
- Pose/velocity fusion is done by [AdaptiveEKFLocalizer.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/FusionLocalizer/AdaptiveEKFLocalizer.java):
  odometry provides the prediction, MT1 provides the observation, and `getStdDevs()` is used to build the adaptive measurement noise R.
- The HIVE state observation must pass through the EKF to [RobotPosition.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/RobotPosition/RobotPosition.java)
  for continuous estimation, and to the autonomous state machine (see the LAUNCH / WAIT\_CELL / EAT\_CELL decisions in [Plan.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Plan.md)).

***

## 2. botpose Fetch (Implemented)

### 2.1 Data Link

Each frame calls `update()` (`LIMELIGHT -> LLResult`):

```java
latestResult = limelight.getLatestResult();   // latest frame cached inside the Limelight 3A
if (latestResult == null || !latestResult.isValid()) { valid = false; return; }
botpose = latestResult.getBotpose();          // robot pose estimated by MegaTag1 (Pose3D)
```

`getBotpose()` returns the field pose estimated by Limelight from the **currently loaded fmap**, with fields defined as:

| Field              | Type                 | Description                    |
| :----------------- | :------------------- | :----------------------------- |
| `getPosition()`    | `Position`           | Translation, in **meters**     |
| `getOrientation()` | `YawPitchRollAngles` | Attitude, angles in **degrees** |

### 2.2 Units and Frame Conversion

The FTC side uniformly uses **inches / radians**, with conversion constant `M_TO_INCH = 39.37007874`:

$$
x_{\text{in}} = x_{\text{m}} \cdot 39.3701,\quad
y_{\text{in}} = y_{\text{m}} \cdot 39.3701,\quad
\theta_{\text{rad}} = \text{toRadians}(\text{yaw}_{\text{deg}})
$$

Frames use the FTC standard field frame: origin at the field center, x right, y forward, z up,
and the corresponding Road Runner 2D pose is `Pose2d(x, y, heading)`.

### 2.3 Quality Metrics (for EKF Adaptive R and Gating)

| Cached field      | `LLResult` interface                      | Purpose                                                        |
| :---------------- | :---------------------------------------- | :------------------------------------------------------------- |
| `stdDevs[6]`      | `getStddevMt1()`                          | `{x, y, z, roll, pitch, yaw}` (meters/degrees), builds the 3x3 R matrix |
| `tagCount`        | `getBotposeTagCount()`                    | Tag count; inflate R when few tags are seen; `isReliable` requires >= 2 |
| `avgDist`         | `getBotposeAvgDist()`                     | Average distance (meters); inflate R quadratically at long range |
| `avgArea`         | `getBotposeAvgArea()`                     | Average area, auxiliary imaging-quality indicator              |
| `span`            | `getBotposeSpan()`                        | Tag span (meters); larger span means more accurate attitude    |
| `maxFiducialSkew` | iterate `getFiducialResults()` for `getSkew()` | Per-tag skew; larger means blurrier image (prone to flip ambiguity) |
| `timestamp`       | `getTimestamp()`                          | Pose timestamp (seconds, Limelight clock), for EKF alignment   |
| `captureLatency`  | `getCaptureLatency()`                     | Capture latency (**milliseconds**, SDK convention), for debugging vision lag |

Composite uncertainty derived from stdDev:

$$
\text{ambiguity} = \sqrt{\sigma_x^2 + \sigma_y^2}\ (\text{meters}),\qquad
\text{angularAmbiguity} = \text{toRadians}(\sigma_{\text{yaw}})
$$

***

## 3. HIVE Angle Estimation and True Pose Recovery (Implemented)

For the algorithm derivation see [OneMap\_MT1Localizer.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/OneMap_MT1Localizer.md);
only the formulas and decision flow needed for implementation are given here.

### 3.1 Problem Statement

- Limelight loads the HIVE **horizontal reference state** ($\theta = 0$) map;
- The actual HIVE has rotated by $\theta$ about the pivot (through $(0,0,h)$ with direction $\mathbf{u} = (0,1,0)$);
- Hence the botpose returned by Limelight is the "rotated" erroneous pose $T' = T_{\text{rot}}(\phi)\,T$, where $\phi = -\theta$;
- Given **a single map**, solve for $\phi$ from $T'$ and recover the true pose $T$.

Denote the botpose translation by $p' = (x', y', z')$ (imperial units) and its attitude by $R'$.

### 3.2 Solving for $\phi$: Using the Robot Ground-Contact Constraint

The robot moves on the ground with true height $z \approx 0$; substituting into the $z$-component equation gives a standard trigonometric equation:

$$
A\sin\phi + B\cos\phi = C,\qquad A = x',\quad B = z' - h,\quad C = -h
$$

Its solution is

$$
\phi = \arcsin\!\left(\frac{C}{\sqrt{A^2+B^2}}\right) - \operatorname{atan2}(B, A)
$$

Implementation points:

1. **Solvability check**: if $\left|C\right| > \sqrt{A^2+B^2}$, this frame has no solution; drop **this frame's HIVE observation**.
   Note `isValid()` and `getPose()` are **unaffected**: then `isHiveEstimated() = false`,
   and `getPose()` falls back to the uncorrected raw pose (contaminated pose; consumers must gate it themselves).
2. **Root selection**: the trigonometric equation has two solution families
   $\phi_1 = \alpha - \operatorname{atan2}(B,A)$ and $\phi_2 = \pi - \alpha - \operatorname{atan2}(B,A)$
   ($\alpha = \arcsin(C/R)$), always differing by $\pi - 2\alpha$; normally only one family falls in the window.
   Filter by $|\phi| \le$ `hiveCellUpAngleDeg`; if both families are inside the window, take the one with **smaller** $|\phi|$.
   This class is stateless and does not use the previous frame's $\phi$ for continuity selection.
3. **Attitude cross-check (degenerate-frame discrimination)**: $\phi$ has two independent information paths --
   the position path (equation above, whose only carrier is $z'$) and the attitude path:
   $$
   R' = R_y(\phi)\,R,\qquad \text{robot on the ground} \Rightarrow pitch' \approx \arcsin\!\big(\sin\phi \cdot \cos(yaw')\big)
   $$
   If the two differ by more than `hivePitchCheckTolDeg`, the frame is judged a $z'$ degenerate frame and **the whole frame is dropped**.
   > Why this is required: with the estimation model $F(\phi) = x'\sin\phi + (z'-h)\cos\phi + h$ we always have $F(0) = z'$,
   > so when $|z'| \approx 0$ the position-path solution necessarily degenerates to $\phi = 0$ ($\theta = 0$), independent of the true HIVE tilt;
   > while `ambiguity` / tag count and other metrics of that frame may all look normal, so normal quality gates cannot remove it.
   > The attitude path does not depend on $z'$ and can serve as an independent arbiter.
   > Limitation: this assumes the robot sits level on the ground (it mis-rejects on a ramp); when heading is near $\pm 90^\circ$, $\cos(yaw') \to 0$,
   > the attitude path itself carries no information about this rotation and the criterion loses its discriminative power.
4. **No time filtering**: EMA / median filtering of $\phi$ is owned by upper layers (this class only outputs per-frame observations).

> Note: the earlier "ground residual check" (back-computing $z$ from $\phi$ and comparing against a tolerance) is identically 0 mathematically --
> since $\phi$ is the analytic root of $A\sin\phi + B\cos\phi = C$, the back-computed $z \equiv C + h = 0$ -- and has no discriminative power;
> it has been replaced by the attitude cross-check in item 3 above.

### 3.3 Position and Attitude Correction

Position (recover $p$ from $p'$):

$$
\begin{aligned}
x &= x'\cos\phi - z'\sin\phi + h\sin\phi \\
y &= y' \\
z &= x'\sin\phi + z'\cos\phi + h(1-\cos\phi)\ \ (\approx 0,\ \text{for checking})
\end{aligned}
$$

Attitude (recover $R$ from $R'$):

$$
R = R_y(-\phi)\,R',\qquad
R_y(-\phi) = \begin{bmatrix} \cos\phi & 0 & -\sin\phi \\ 0 & 1 & 0 \\ \sin\phi & 0 & \cos\phi \end{bmatrix}
$$

Only the 2D pose is exposed; yaw is extracted from the corrected rotation matrix. The first column of $R'$ is the direction of the robot x axis in the field frame
(SDK convention: intrinsic yaw -> pitch -> roll, so $R_{00} = \cos\psi'\cos P'$, $R_{10} = \sin\psi'\cos P'$, $R_{20} = -\sin P'$):

$$
\text{yaw} = \operatorname{atan2}\big(R_{10},\ \cos\phi\,R_{00} - \sin\phi\,R_{20}\big)
$$

> An earlier document wrote $\text{yaw} \approx \operatorname{atan2}(R_{21}, R_{11})$, which is always 0 when the robot is level; that was a typo and is implemented as above.

### 3.4 HIVE State Decision

Decided **instantaneously per frame** from $\theta = -\phi$ (actual HIVE tilt), **without hysteresis** (this class keeps no history;
hysteresis/state hold is owned by the upper-layer RobotPosition):

| Condition                                                        | State           | Description                                                        |
| :--------------------------------------------------------------- | :-------------- | :----------------------------------------------------------------- |
| No valid observation in this frame (no tags / no solution / attitude cross-check failed) | `UNKNOWN`       | Upper layer keeps the last known state, no transition              |
| $\lvert\theta\rvert \le \theta_{\text{down}}$                    | `MIDDLE`        | HIVE level/middle (CELL down), $\phi$ of both paths agrees         |
| $\theta_{\text{down}} < \lvert\theta\rvert < \theta_{\text{up}}$ and $\theta > 0$ | `AUDIENCE_UP`   | HIVE stably on one side; which side is up follows the sign convention of `hivePositiveAngleIsAudienceUp` |
| $\theta_{\text{down}} < \lvert\theta\rvert < \theta_{\text{up}}$ and $\theta < 0$ | `AUDIENCE_DOWN` | The other side is up                                                |

Here $\theta_{\text{down}}$ = `hiveCellDownAngleDeg` and $\theta_{\text{up}}$ = `hiveCellUpAngleDeg`.
Frames with $\lvert\theta\rvert \ge \theta_{\text{up}}$ are already dropped at the root-selection stage (Sec. 3.2-2) and never reach this classification.

> The two thresholds serve two duties each: `hiveCellUpAngleDeg` is the **upper bound** of the root-selection window (and a direct rejection threshold),
> `hiveCellDownAngleDeg` is the **boundary** between MIDDLE and "stably on one side". When calibrating on the real robot, set the former slightly above the maximum actual tilt,
> and the latter to the upper bound of residual jitter when "truly level".

### 3.5 Usage Constraints

1. **Feed only own-side tags**: filter via `SetFiducialIDFiltersOverride` / pipeline so only own-side HIVE tags are kept.
   Mixing red and blue HIVE tags makes rotation axes inconsistent and breaks the estimation (see the end of the OneMap document).
2. **Keep only the single horizontal-reference-state map**; no need to upload fmaps dynamically by tilt (this is an advantage over the two-map switching scheme in [map.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/VisionLocalizer/map.md)).
3. **$h$** **must be calibrated**: the pivot-center height above the field ground (inches); its error feeds linearly into the $x$ correction.
4. **Visibility range**: valid only when tags ahead of the HIVE are visible, per the "localization only needs to cover the area ahead of the HIVE, not the whole field" convention in [Plan.md](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/OpModes/Plan.md).
5. **Limelight `snap to ground` must be set to `No`**: when that option is on, Limelight clamps the estimated pose onto the ground plane
   (equivalent to $z' \equiv 0$), the tilt information in the position path is completely erased, and the estimation **always outputs** $\theta = 0$.
   This was a confirmed failure cause during on-robot debugging; it is a configuration issue, not an algorithm issue.
6. **The robot must sit level on the ground**: the attitude cross-check (Sec. 3.2-3) assumes true pitch ~= 0; a robot on a ramp is misjudged as a degenerate frame.

***

## 4. External Interfaces

### 4.1 Implemented (current `.java`)

`Localizer` interface:

| Method            | Return           | Description                                    |
| :---------------- | :--------------- | :--------------------------------------------- |
| `update()`        | `PoseVelocity2d` | Pull the latest frame; always returns zero velocity (vision provides no velocity) |
| `getPose()`       | `Pose2d`         | Global pose (inch, inch, radian); returns `(0,0,0)` when invalid |
| `setPose(Pose2d)` | `void`           | No-op (a vision localizer is not writable)     |

Extended read interfaces:

| Method                                                              | Return           | Description                                                                        |
| :------------------------------------------------------------------ | :--------------- | :--------------------------------------------------------------------------------- |
| `getPoseArray()`                                                    | `double[3]`      | `{x, y, theta}` (inch, inch, radian)                                               |
| `getBotpose()`                                                      | `Pose3D`         | Raw 6DOF pose (meters/degrees)                                                     |
| `getStdDevs()`                                                      | `double[6]`      | Copy of `{x,y,z,roll,pitch,yaw}` (meters/degrees)                                  |
| `getAmbiguity()`                                                    | `double`         | Planar position uncertainty $\sqrt{\sigma_x^2+\sigma_y^2}$ (meters); `Double.MAX_VALUE` when invalid |
| `getAngularAmbiguity()`                                             | `double`         | yaw uncertainty (radians)                                                          |
| `getTagCount()` / `getAvgDist()` / `getAvgArea()` / `getSpan()`     | `int` / `double` | Quality metrics                                                                    |
| `getMaxFiducialSkew()`                                              | `double`         | Maximum per-tag skew                                                               |
| `getTagIds()`                                                       | `String`         | All fiducial IDs seen in this frame (comma-separated), for debugging: confirm only own-side tags are present |
| `getTimestamp()` / `getCaptureLatency()`                            | `double`         | Timestamp (seconds, Limelight clock) / capture latency (**milliseconds**)          |
| `isValid()`                                                         | `boolean`        | Whether a valid result was pulled for the current frame (**no guarantee** that HIVE estimation succeeded) |
| `isReliable(double threshold)`                                      | `boolean`        | `valid && tagCount >= 2 && ambiguity < threshold`, for EKF update gating           |

HIVE estimation observation interfaces:

| Method                          | Return      | Description                                                                                   |
| :------------------------------ | :---------- | :-------------------------------------------------------------------------------------------- |
| `getHiveAngle()`                | `double`    | HIVE tilt $\theta = -\phi$ (radians), `Double.NaN` when unsolved                               |
| `getHiveState()`                | `HiveState` | `MIDDLE` / `AUDIENCE_UP` / `AUDIENCE_DOWN` / `UNKNOWN` (Sec. 3.4), no hysteresis               |
| `isHiveEstimated()`             | `boolean`   | Whether $\theta$ was solved for this frame; distinguishes "truly MIDDLE" from "not seen / rejected" |
| `getHivePitchCheckErrDeg()`     | `double`    | Attitude cross-check deviation (degrees); frames exceeding `hivePitchCheckTolDeg` are already dropped, `NaN` when no observation (Sec. 3.2-3) |
| `getRawPose()`                  | `Pose2d`    | Limelight raw (uncorrected) pose, for debug comparison                                        |
| `getRawZIn()` / `getRawPitch()` | `double`    | Raw-pose $z'$ (inches) / pitch (radians), for degenerate-frame diagnosis (Sec. 3.2-3)          |

### 4.2 Not Yet Implemented

| Method                                                 | Return | Description                                                                            |
| :----------------------------------------------------- | :----- | :------------------------------------------------------------------------------------- |
| `setAlliance(boolean isRed)` or `setTagIds(int[] ids)` | `void` | Keep only own-side tags inside this class; currently relies on Limelight `SetFiducialIDFiltersOverride` / fmap filtering |
| `setMode(...)`                                         | `void` | Pass the HIVE state through the EKF to RobotPosition (see the todo in `AdaptiveEKFLocalizer`) |

> Parameters ($h$, the two angle thresholds, check tolerance, sign convention) are not injected via setters; they are uniformly provided by
> static fields in [HypParams.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Parameter/HypParams.java) for live tuning on the FTC Dashboard.

Interface conventions:

- `getPose()` returns the **corrected true pose** and still satisfies the `Localizer` contract (inches/radians), so the EKF benefits without changes;
  but on HIVE-estimation failure it **falls back to the uncorrected contaminated pose**, so consumers must gate with
  `isValid() && isHiveEstimated()` (see Sec. 5);
- Validity of `getHiveAngle()` / `getHiveState()` is guarded by `isHiveEstimated()`;
  the upper-layer state machine uses it to distinguish `MIDDLE` (tilt genuinely small) from `UNKNOWN` (not seen / rejected).

***

## 5. Collaboration Flow with the EKF (Adaptive R and Gating)

Call order in [AdaptiveEKFLocalizer.update()](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/FusionLocalizer/AdaptiveEKFLocalizer.java#L182-L218):

1. Odometry `odom.update()` -> velocity;
2. `adaptQ(dt)` -> IMU-shock/slope adaptive process noise;
3. `ekf.predict(...)`;
4. `mt1.update()` ->
   - Continue only if `mt1.isValid() && mt1.isHiveEstimated()`; otherwise this frame's `getPose()` is the uncorrected contaminated pose and the whole frame is dropped;
   - `adaptR()` builds the 3x3 R from `getStdDevs()` + `getAvgDist()` + `getTagCount()`;
   - `ekf.gateVision(...)` removes outlier observations by Mahalanobis distance;
   - `ekf.update(mt1.getPose(), mt1.getTimestamp())`.
5. Pass through `mt1.getHiveAngle()` / `mt1.getHiveState()` (not yet implemented, see the todo in `AdaptiveEKFLocalizer`).

The comparison localizers (`EKFLocalizer` / `UKFLocalizer`) do no Mahalanobis-distance gating, but **likewise require**
`isValid() && isHiveEstimated()`; otherwise comparison results contain contaminated poses and lose comparability.

Therefore changes to MT1Localizer **must not break** these three output groups: `getPose()` / `getStdDevs()` / `getTimestamp()`.

***

## 6. Parameters and Todo List

Parameters to calibrate/configure (all in [HypParams.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Parameter/HypParams.java), adjustable live on the Dashboard):

| Parameter                     | Meaning                                                   | Notes                                                        |
| :---------------------------- | :-------------------------------------------------------- | :----------------------------------------------------------- |
| `hivePivotHeightIn`           | Pivot height above ground $h$ (inches)                    | Must be calibrated; its error feeds linearly into the $x$ correction |
| `hiveCellDownAngleDeg`        | Tilt upper bound of MIDDLE (degrees)                      | Also the lower bound of "stably on one side"                 |
| `hiveCellUpAngleDeg`          | Tilt upper bound of stably-on-one-side (degrees), also the **root-selection window upper bound** | Frames above it are rejected directly; must be > `hiveCellDownAngleDeg` |
| `hivePitchCheckTolDeg`        | Attitude cross-check tolerance (degrees)                  | Position-path vs attitude-path $\phi$ difference above this value -> degenerate frame |
| `hivePositiveAngleIsAudienceUp` | Tilt sign convention (boolean)                           | Decides which side-up $\theta > 0$ corresponds to; calibrate on the real robot |

Todos:

- [x] Implement $\phi$ solving and root selection in Sec. 3.2;
- [x] Implement degenerate-frame discrimination in Sec. 3.2-3 (attitude cross-check), implement position/attitude correction in Sec. 3.3, and switch `getPose()` to the corrected pose;
- [x] Implement HIVE state decision in Sec. 3.4 (no hysteresis) and the HIVE observation interfaces in Sec. 4.1;
- [x] Add $\theta$ / HIVE state / before-and-after-correction poses / Dashboard trajectory comparison in `MT1Test.java` / `FusionTestOpMode.java`;
- [ ] Pass the HIVE state through in `AdaptiveEKFLocalizer` to [RobotPosition.java](file:///f:/github/BIOBUZZ/TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Processors/RobotPosition/RobotPosition.java), and implement hysteresis / state hold in upper layers;
- [ ] Calibrate each parameter in Sec. 6 on the real robot ($h$ and the two angle thresholds);
- [ ] `setAlliance` / `setTagIds` tag filtering (currently relies on Limelight-side configuration, see Sec. 3.5-1).
