# FusionLocalizer Theory

## Overview

`FusionLocalizer` is a multi-sensor fusion localizer for FTC robotics competition. It provides **two filter implementations**:

| Implementation | Class names | Filter | Characteristics |
|------|------|--------|------|
| **EKF** | `EKF` / `EKFLocalizer` / `AdaptiveEKFLocalizer` | Extended Kalman filter | Jacobian linearization, cheap, mature and stable |
| **UKF** | `UKF` / `UKFLocalizer` / `AdaptiveUKFLocalizer` | Unscented Kalman filter | Sigma-point unscented transform, no Jacobian needed, more accurate |

It complements **odometry** (high rate, accurate short-term, drifts long-term) with **Limelight MegaTag1 vision** (low frame rate, absolute position, occasionally unavailable), producing a smooth, robust global pose estimate.

Core idea: **use odometry for inter-frame prediction, Limelight for absolute correction, the Hub IMU for abnormal-motion detection, and ambiguity for vision confidence.**

### D2 vs. D3 Modes

| Feature | D2 mode (default) | D3 mode |
|------|---------------|---------|
| Odometry | `PinpointLocalizer` (standard 2D) | `PinpointD3Localizer` (3D ramp compensation) |
| Ramp compensation | None | Hub IMU pitch/roll → project velocity onto the horizontal plane |
| Required hardware | Pinpoint + Limelight | Pinpoint + Limelight + Hub IMU |
| Use case | Flat field | Field with ramps |

> All localizer classes (`EKFLocalizer`, `AdaptiveEKFLocalizer`, `UKFLocalizer`, `AdaptiveUKFLocalizer`) support D2/D3 mode switching. The EKF and UKF APIs are fully compatible drop-in replacements for each other.
>
> For the 5D accelerometer-driven variants (`AdaptiveEKF5DLocalizer` / `AdaptiveUKF5DLocalizer` and their simplified D2 `*_D2` versions), see [Theory5D.md](./Theory5D.md).

> **D3 ramp-compensation sign convention**: D3 mode projects velocity with pitch positive when leaning forward and roll positive when the left side lifts; the cross term carries a **minus sign** (`v_x^{horiz} = v_x^{body}·cosθ - v_y^{body}·sinθ·sinφ`). For the full derivation, see [`D3Localizer/Theory.md`](../D3Localizer/Theory.md).

---

## 1. System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                  Per-frame update() flow                   │
├──────────────────────────────────────────────────────────┤
│                                                          │
│   Odometry (D2: PinpointLocalizer / D3: PinpointD3Localizer) │
│     └─→ body velocity (vx, vy, ω)  [in/s]                      │
│        │                                                  │
│        ▼                                                  │
│   Hub IMU (BHI260IMU)                                     │
│     ├─ D2: roll/pitch angular accel → field frame → cross map → adaptQ_x, adaptQ_y│
│     │  D3: roll/pitch angular rate → field frame → cross map → adaptQ_x, adaptQ_y │
│     └─ yaw angular accel → adaptQ_θ                             │
│        │                                                  │
│        ▼                                                  │
│   adaptQ(dt) ──→ SimpleMatrix Q (3x3, in²/s)              │
│        │                                                  │
│        ▼                                                  │
│   EKF/UKF.setQ(Q) + EKF/UKF.predict(vx, vy, ω, timestamp)   │
│        │           state x, y in inches                    │
│        ▼                                                  │
│   Limelight MT1 ──→ {x_m, y_m} [m] ──→ ×39.37 ──→ {x, y} [in] │
│        │                                                  │
│        ▼                                                  │
│   adaptR() ──→ SimpleMatrix R (3x3, in²)                  │
│        │                                                  │
│        ▼                                                  │
│   EKF/UKF.setR(R) + EKF/UKF.update(x, y, θ, timestamp)      │
│                    (only when vision is valid)                           │
└──────────────────────────────────────────────────────────┘
```

### State Definition

```
State vector:  x = [x, y, θ]ᵀ
  - x, y: robot position in the field frame (inches)
  - θ:    global heading (radians, [-π, π])

Control input:  u = [vx, vy, ω]ᵀ  (robot-frame velocity from Pinpoint)
  - vx: forward velocity (in/s)
  - vy: strafe velocity (in/s)
  - ω:  angular velocity (rad/s)

Measurement:     z = [x_m, y_m, θ_m]ᵀ  (global pose measured by Limelight)
  - Raw units: meters -> converted to inches before entering the filter
```

---

## 2. EKF Predict Step

### 2.1 Nonlinear Motion Model

Transform the body-frame velocity from Pinpoint into the field frame:

```
x' = x + Δt · (vx·cosθ - vy·sinθ)
y' = y + Δt · (vx·sinθ + vy·cosθ)
θ' = θ + Δt · ω
```

### 2.2 Jacobian Matrix

Take partial derivatives of the state propagation function with respect to the state, for covariance propagation:

```
      ┌                                        ┐
      │ 1    0    Δt·(-vx·sinθ - vy·cosθ)      │
A =   │ 0    1    Δt·( vx·cosθ - vy·sinθ)      │
      │ 0    0    1                             │
      └                                        ┘
```

### 2.3 Covariance Propagation

```
P' = A · P · Aᵀ + Q · Δt
```

Here `Q · Δt` captures how process noise accumulates over time: larger Δt means more growth in prediction uncertainty.

---

## 3. EKF Update Step

### 3.1 Innovation

```
y = z - H·x          (H = I, measurements map directly onto the state)
```

Normalize the heading innovation to `[-π, π]` to avoid errors from ±2π jumps.

### 3.2 Kalman Gain

```
S = H·P·Hᵀ + R
K = P·Hᵀ · S⁻¹
```

### 3.3 State and Covariance Update

```
x = x + K·y
P = (I - K·H)·P
```

---

## 2b. UKF Predict Step

The UKF uses no Jacobian; instead it propagates the nonlinear distribution directly through the **Unscented Transform**.

### 2b.1 UKF Parameters

The UKF presets three parameters controlling the sigma-point spread:

| Parameter | Value | Meaning |
|------|-----|------|
| **α** (alpha) | 1.0 | Sigma-point spread factor, controlling distance from the mean. Smaller α packs points tighter |
| **β** (beta) | 2.0 | Optimal parameter; β=2 is optimal for Gaussian distributions |
| **κ** (kappa) | 0.0 | Secondary scaling parameter, usually 0 or 3-n |
| **n** | 3 | State dimension = [x, y, θ] |
| **λ** | 0 | λ = α²(n+κ) - n = 1·3 - 3 = 0 |
| **n+λ** | 3 | Scale factor for sigma-point deviations and weights |

### 2b.2 Sigma-Point Generation

Generate **2n+1 = 7** sigma points from the current state estimate `x̂` and covariance `P`:

```
1. Compute the scaled covariance: S = (n+λ) · P = 3 · P

2. Cholesky factorization: S = L · Lᵀ
   where L is a lower-triangular matrix (3×3), computed with EJML DecompositionFactory_DDRM.chol()

3. Generate the sigma points:
   X₀   = x̂                              (mean point)
   Xᵢ   = x̂ + L_colᵢ                     (i = 1, 2, 3)
   Xᵢ₊₃ = x̂ - L_colᵢ                     (i = 1, 2, 3)
```

**Sigma-point weights**:

| Index | Mean weight wm | Covariance weight wc | Notes |
|------|------------|--------------|------|
| 0 | λ/(n+λ) = **0** | λ/(n+λ) + (1-α²+β) = **2** | Center point |
| 1..6 | 1/(2(n+λ)) = **1/6** | 1/(2(n+λ)) = **1/6** | Outer points |

> **Note**: wc[0] = 2 is expected. For n=3, α=1, β=2, the weights sum to Σwc = 2 + 6×(1/6) = 3, satisfying Σwc = n + λ.

### 2b.3 Nonlinear Motion-Model Propagation

Pass every sigma point through the same nonlinear motion model:

```
X̂ᵢ[0] = Xᵢ[0] + Δt · (vx·cos(Xᵢ[2]) - vy·sin(Xᵢ[2]))
X̂ᵢ[1] = Xᵢ[1] + Δt · (vx·sin(Xᵢ[2]) + vy·cos(Xᵢ[2]))
X̂ᵢ[2] = Xᵢ[2] + Δt · ω
```

### 2b.4 Weighted Mean (circular mean)

```
x̂̄[0] = Σ wm[i] · X̂ᵢ[0]       (position: plain weighted mean)
x̂̄[1] = Σ wm[i] · X̂ᵢ[1]       (position: plain weighted mean)
x̂̄[2] = atan2( Σ wm[i]·sin(X̂ᵢ[2]), Σ wm[i]·cos(X̂ᵢ[2]) )   (heading: circular mean)
```

**Why a circular mean?** Heading is periodic ([-π, π]). If sigma points straddle ±π, a plain arithmetic mean gives the wrong answer. The circular mean `atan2(Σsin, Σcos)` handles angle wrap-around correctly.

### 2b.5 Covariance Propagation

```
P̄ = Σ wc[i] · (X̂ᵢ - x̂̄)(X̂ᵢ - x̂̄)ᵀ + Q · Δt
```

In each sigma-point deviation, normalize the heading difference to `[-π, π]` before forming the outer product.

---

## 3b. UKF Update Step

Because the measurement model is linear (`H = I`, measurements map directly onto the state), the UKF update could be simplified, but it still runs the full unscented transform for numerical consistency.

### 3b.1 Measurement Sigma Points

Since `H = I`, the measurement sigma points `Zᵢ` equal the state sigma points `Xᵢ`:

```
Zᵢ = H · Xᵢ = Xᵢ
```

### 3b.2 Innovation

```
Predicted measurement mean ẑ = Σ wm[i] · Zᵢ  (= x̂̄, same as the state mean)
Innovation y_innov = z - ẑ

Normalize the heading innovation to [-π, π]
```

### 3b.3 Innovation and Cross Covariances

```
S = Σ wc[i] · (Zᵢ - ẑ)(Zᵢ - ẑ)ᵀ + R    (innovation covariance)
Pxz = Σ wc[i] · (Xᵢ - x̂̄)(Zᵢ - ẑ)ᵀ      (cross covariance)
```

### 3b.4 Kalman Gain and State Update

```
K = Pxz · S⁻¹
x̂ = x̂̄ + K · y_innov
```

### 3b.5 Joseph-Form Covariance Update

The UKF updates covariance in **Joseph form** instead of the EKF `(I-KH)` form:

```
P = P̄ - K · S · Kᵀ
```

**Why Joseph form?**

| Form | Formula | Properties |
|------|------|------|
| EKF standard | `P = (I - K·H)·P` | Concise, but rounding error can leave P asymmetric |
| Joseph form | `P = P - K·S·Kᵀ` | Symmetric by construction, keeps positive semi-definiteness, more stable |

After the Joseph update the UKF additionally enforces symmetry:

```
P = (P + Pᵀ) / 2
```

### 3b.6 Cholesky Factorization and Numerical Stability

Sigma-point generation relies on the Cholesky factorization `L·Lᵀ = (n+λ)·P`. When rounding error makes the covariance `P` non-positive-definite, Cholesky fails. The UKF uses a **three-level regularization fallback**:

```
Attempt 1: factor scale·P directly
   ↓ on failure
Attempt 2: factor scale·P + 1e-6 · I (tiny regularization)
   ↓ still failing
Attempt 3: factor scale·P + 1e-3 · I (stronger regularization)
   ↓ still failing
Fallback: return diagonal matrix sqrt(trace(scale·P)/3) · I
```

This policy guarantees valid sigma points in every case, so numerical issues can never crash the filter.

---

## 3c. EKF vs. UKF

### 3c.1 Algorithm differences at a glance

| Aspect | EKF | UKF |
|------|-----|-----|
| **Nonlinearity handling** | First-order Taylor expansion (Jacobian `A = ∂f/∂x`) | Sigma-point unscented transform, exact to second moment (third order for Gaussian) |
| **Jacobian matrix** | Must be derived by hand, `A[0][2]=Δt·(-vx·sinθ-vy·cosθ)` | Not needed |
| **State propagation** | 1 pass | 7 passes (2n+1 sigma points) |
| **Covariance propagation** | `P = A·P·Aᵀ + Q·dt` | Sigma-point weighted outer products + Q·dt |
| **Kalman gain** | `K = P·(P+R)⁻¹` | `K = Pxz·S⁻¹` |
| **Covariance update** | `P = (I-K·H)·P` | Joseph form `P = P - K·S·Kᵀ` |
| **Angle mean** | None (single state point) | Circular mean `atan2(Σsin, Σcos)` |
| **Matrix factorization** | None | Cholesky factorization (3×3) |
| **Compute cost** | ~15 3×3 matrix multiplies | ~15 3×3 matrix multiplies + 7 state propagations + Cholesky |
| **FTC frame time** | < 50 μs | < 200 μs (still far below one frame budget) |

### 3c.2 When to use the UKF

| Scenario | Recommended | Why |
|------|------|------|
| Fast spin (omega > 2π rad/s) | **UKF** | EKF linearization error accumulates as cos/sin change rapidly |
| Recovery after a collision | **UKF** | UKF propagates collision-disturbed covariance more accurately |
| Flat field, low speed | EKF | Comparable accuracy, EKF is simpler |
| Extremely tight compute budget | EKF | UKF pays a small Cholesky cost |
| Debugging/teaching | EKF | Jacobian can be verified by hand, easier to understand |

### 3c.3 Why the accuracy differs

The EKF first-order Taylor-expands the nonlinear motion model `f(x)` at the estimate:

```
EKF:  f(x̂ + δx) ≈ f(x̂) + A·δx
```

This drops the curvature of `f`. When `θ` changes a lot (e.g. rotating 0.5 rad within 10ms), the gap between `cos(θ+δ)` and `cos(θ) - sin(θ)·δ` reaches ~0.12 rad, which turns into a ~0.12·v·Δt inch error in position propagation.

The UKF directly samples the nonlinear function with 7 sigma points, capturing the curvature of `cos/sin`:

```
UKF:  P̄ = Σ wc[i] · (f(Xᵢ) - x̂̄)(f(Xᵢ) - x̂̄)ᵀ
```

This is equivalent to statistical linear regression of `f(x)`: it minimizes linearization error over the whole distribution instead of at a single point.

### 3c.4 API compatibility

The EKF and UKF expose identical public APIs:

```java
// Both filters are used exactly the same way
EKF filter1 = new EKF(x, y, theta);
UKF filter2 = new UKF(x, y, theta);

filter1.predict(vx, vy, omega, timestamp);  // identical to filter2
filter1.update(xMeas, yMeas, thetaMeas, timestamp);  // identical to filter2
filter1.setQ(SimpleMatrix Q);  // identical to filter2
filter1.setR(SimpleMatrix R);  // identical to filter2

// Localizer wrappers are also drop-in replacements
AdaptiveEKFLocalizer ekfLoc = new AdaptiveEKFLocalizer(...);
AdaptiveUKFLocalizer ukfLoc = new AdaptiveUKFLocalizer(...);  // only the type name differs
```

The adaptive Q/R logic (IMU impact detection, D2/D3 policy, Limelight stdDev three-band mapping, boost/decay regulation) is **identical** in both implementations, because the adaptive logic only scales the Q/R matrix values and never depends on filter internals.

---

## 4. Adaptive Q Tuning (Hub IMU)

### 4.1 Motivation

When the robot collides, stops hard, or gets hit by another robot, **odometry momentarily produces unreliable displacement** (encoder slip, IMU oscillation). If the EKF still trusted the odometry prediction at that moment, the fused pose would drift away from the true position.

Additionally, in **D3 mode**, slope changes while crossing a ramp can lift the pinpoint off the ground, which also calls for a larger Q.

**Solution**: use the Hub IMU (BHI260IMU) to detect abnormal motion and scale the Q multiplier **independently per axis**. D2 and D3 use different detection strategies. The uncertain direction is perpendicular to the rotation axis: pitch about body Y lifts wheels along X, roll about body X lifts wheels along Y.

### 4.2 D2 Mode: angular acceleration only (impact detection)

The Hub IMU cannot report linear acceleration directly, so roll/pitch angular acceleration serves as an indirect indicator that the robot was hit; the body-frame disturbance is then rotated into the field frame using the current heading.

**Body-frame definitions**:
- roll = `xRotationRate` → rotation about body X (left/right)
- pitch = `yRotationRate` → rotation about body Y (forward/back)
- yaw = `zRotationRate` → rotation about body Z (vertical)

**Cross-mapping principle**: the uncertain direction is perpendicular to the rotation axis. Pitch about body Y lifts wheels along X, roll about body X lifts wheels along Y.

| Axis | Sensor | Quantity | Physical meaning |
|---|---|---|---|
| **x** | Hub IMU pitch angular acceleration | `\|fieldY\|` (rad/s²) | Forward/backward hit (pitch about Y lifts along X; take the fieldY component) |
| **y** | Hub IMU roll angular acceleration | `\|fieldX\|` (rad/s²) | Side hit (roll about X lifts along Y; take the fieldX component) |
| **θ** | Hub IMU yaw angular acceleration | `\|Δω_z/Δt\|` (rad/s²) | Hit in the rotation direction |

Data sources:
- `getRobotAngularVelocity(AngleUnit.RADIANS).xRotationRate` → roll rate
- `getRobotAngularVelocity(AngleUnit.RADIANS).yRotationRate` → pitch rate
- `getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate` → yaw rate

Body-frame → field-frame rotation (Rz(θ)):
```
rollAccel  = (rollRate  - lastRollRate)  / dt
pitchAccel = (pitchRate - lastPitchRate) / dt

fieldX = rollAccel · cos(heading) - pitchAccel · sin(heading)
fieldY = rollAccel · sin(heading) + pitchAccel · cos(heading)

// Cross mapping: pitch about Y → lifts along X → X uncertainty takes fieldY
//                roll about X → lifts along Y → Y uncertainty takes fieldX
qBoostX = updateBoost(qBoostX, |fieldY|, ANGULAR_ACCEL_THRESHOLD)
qBoostY = updateBoost(qBoostY, |fieldX|, ANGULAR_ACCEL_THRESHOLD)
```

### 4.3 D3 Mode: angular velocity (slope-change detection)

D3 mode uses `PinpointD3Localizer` for 3D ramp compensation, so odometry already projects velocity onto the slope. But when the slope **changes** (e.g. transitioning from a ramp back to flat ground), the odometry velocity estimate can still glitch briefly. D3 therefore uses **angular velocity** to detect slope changes, again cross-mapped onto each axis:

| Signal | Detects | Physical meaning | Boost cap |
|------|----------|----------|------------|
| **Field-frame angular velocity** | `\|fieldVelY\|`, `\|fieldVelX\|` | Slope change (robot tilting/recovering), cross-mapped | 10x |
| **yaw angular acceleration** | `\|Δω_yaw/Δt\|` | Rotation impact | 10x |

> **Important**: yaw rate is unrelated to slope, so it does not participate in D3 slope-change detection. Yaw only uses angular acceleration to detect rotation impacts.

D3 rotates the angular velocity into the field frame and then cross-maps it:

```
// Rotate angular velocity into the field frame (Rz(θ))
fieldVelX = rollRate · cos(heading) - pitchRate · sin(heading)
fieldVelY = rollRate · sin(heading) + pitchRate · cos(heading)

// Cross mapping: pitch about Y → lifts along X → X uncertainty takes fieldVelY
//                roll about X → lifts along Y → Y uncertainty takes fieldVelX
qBoostX = updateBoost(qBoostX, |fieldVelY|, ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX)
qBoostY = updateBoost(qBoostY, |fieldVelX|, ANGULAR_VEL_THRESHOLD, VEL_BOOST_MAX)
```

**Design intent**: after rotating into the field frame and cross-mapping, each axis is judged independently. For example, when the robot climbs a ramp facing forward, the pitch rate maps mostly onto field Y, and cross-mapping raises Q on X; when rolling over a side slope, the roll rate maps mostly onto field X, and cross-mapping raises Q on Y.

### 4.4 Q Tuning Policy

Each axis independently runs the same boost/decay logic:

```
if magnitude > threshold:
    qBoost = min(Q_BOOST_MAX, qBoost × (1 + magnitude / threshold))
else:
    qBoost = max(1.0, qBoost × Q_DECAY)

Q_diag[i] = Q_base × qBoost[i]
```

- **On impact (D2)**: angular acceleration grows qBoost exponentially; harder impacts raise it faster
- **On slope change (D3)**: angular velocity grows qBoost exponentially (max 10x)
- **When settled**: qBoost decays exponentially (×0.85 per frame) back to baseline

Finally `adaptQ(dt)` returns a diagonal 3x3 `SimpleMatrix` passed directly to `EKF/UKF.setQ(SimpleMatrix)`.

### 4.5 Tuning Guide

| Parameter | Recommended | Meaning | Modes |
|------|--------|------|------|
| `ANGULAR_ACCEL_THRESHOLD` | 5.0 rad/s² | roll/pitch angular-acceleration trigger threshold | D2 |
| `JERK_THRESHOLD` | 4.0 rad/s² | yaw angular-acceleration trigger threshold | D2, D3 |
| `ANGULAR_VEL_THRESHOLD` | 1.0 rad/s | roll/pitch angular-velocity trigger threshold (slope change) | D3 |
| `VEL_BOOST_MAX` | 10.0 | max Q multiplier from angular velocity (slope change) | D3 |
| `Q_BOOST_MAX` | 10.0 | max Q multiplier | D2 |
| `Q_DECAY` | 0.85 | per-frame decay factor | D2, D3 |

---

## 5. Adaptive R Tuning (Limelight stdDev)

### 5.1 Motivation

Limelight MegaTag1 accuracy varies greatly across conditions:
- **Many tags, short distance, good lighting** → high accuracy (stddev ~0.01m)
- **Single tag, long distance, large tilt angle** → low accuracy (stddev ~0.1m+)
- **Occlusion, motion blur** → completely unusable

**Solution**: use the per-axis `stddevMt1` components reported by the Limelight firmware to **tune R_x, R_y, R_θ independently**. Uncertainty on each axis only affects that axis's measurement noise.

### 5.2 stdDev Source and Unit Conversion

```
double[] stdDevs = mt1.getStdDevs();  // {x, y, z, roll, pitch, yaw} (meters, degrees)
```

From `LLResult.getStddevMt1()` — standard deviations output directly by the MegaTag1 algorithm inside the Limelight firmware.

**Position vs. angle use different thresholds**:
- **x, y**: stdDev from meters → inches (×39.37), compared against position thresholds
- **θ (yaw)**: stdDev from degrees → radians, compared against angle-specific thresholds

### 5.3 R Tuning Policy (per-axis, no upper cap)

Each axis maps std → R with an **uncapped linear map** (keeps growing linearly past the high threshold instead of saturating at `R_MAX_SCALE`; 9/2 improvement):

```
if std <= LOW:                          R = 0.01
else:  t = (std - LOW) / (HIGH - LOW)
       R = 0.01 × (1 + t × (R_MAX_SCALE - 1))   # uncapped, keeps growing linearly
```

- **Position axes (x, y)**: `std_inch = std_meter × 39.37`, thresholds use `STD_LOW_INCH` / `STD_HIGH_INCH`.
- **Heading axis (θ)**: `std_rad = Math.toRadians(std_yaw_deg)`, thresholds use `STD_LOW_ANGLE` / `STD_HIGH_ANGLE`.

Position-axis R is additionally multiplied by two scale factors to handle vision error blowing up far from AprilTags or with too few tags (9/2 experimental finding):

```
rX     = mapStdToR(stdX_inch)  × distFactor × tagFactor
rY     = mapStdToR(stdY_inch)  × distFactor × tagFactor
rTheta = mapStdToRAngle(stdYaw_rad)      # angle uses no distance/tag factors
```

Scale factor definitions:

- **Distance scale `distFactor`**: `dist = mt1.getAvgDist()` (meters); when `dist > DIST_REF_M`, grows quadratically as `(dist / DIST_REF_M)²`, otherwise 1. Far from tags, vision position error grows roughly with distance squared, hence the quadratic term.
- **Tag-count scale `tagFactor`**: `tags = mt1.getTagCount()`; 1 when `tags >= TAG_REF`; `TAG_REF / tags` (capped at `TAG_SCALE_MAX`) when `0 < tags < TAG_REF`; `TAG_SCALE_MAX` when `tags <= 0`. Fewer tags means a less reliable pose solution, hence larger R.

Finally `adaptR()` returns a diagonal 3x3 `SimpleMatrix` passed directly to `EKF/UKF.setR(SimpleMatrix)`.

### 5.4 Tuning Guide

| Parameter | Recommended | Meaning |
|---|---|---|
| `STD_LOW_INCH` | 2.0 in (≈0.05m) | Position: fully trust vision below this (R=0.01) |
| `STD_HIGH_INCH` | 6.0 in (≈0.15m) | Position: upper edge of the linear growth band (keeps growing linearly past it) |
| `STD_LOW_ANGLE` | 0.035 rad (≈2°) | Heading: fully trust vision below this |
| `STD_HIGH_ANGLE` | 0.175 rad (≈10°) | Heading: upper edge of the linear growth band |
| `R_MAX_SCALE` | 20.0 | std→R linear-map slope (**no longer capped**) |
| `DIST_REF_M` | 1.0 m | Distance-scale reference distance (quadratic growth past it) |
| `TAG_REF` | 2.0 | Tag-count scale reference tag count |
| `TAG_SCALE_MAX` | 4.0 | Max R magnification when too few tags are visible |
| `GATE_THRESHOLD` | 4.0 | Mahalanobis gate threshold (dimensionless, larger is looser) |

### 5.5 Mahalanobis Gating (outlier rejection)

Even with R scaled by distance/tag count, a distant vision solution can still carry huge errors that would drag the filter off if applied. So before every vision update, a **Mahalanobis gate** decides whether the measurement is an outlier:

```
S  = P + R                    (measurement model H = I → innovation covariance)
d² = (z - x̂)ᵀ · S⁻¹ · (z - x̂)
if d² <= GATE_THRESHOLD² :  accept the update
else                      :  reject (vision discarded for this frame)
```

- The heading innovation is normalized to `[-π, π]` before the computation.
- A rejected frame only skips the vision update; predict still runs, and the filter keeps relying on odometry until the next trustworthy measurement.
- This gating logic lives in the filter base class `gateVision()`; each localizer gates first inside the vision branch of `update()` and only calls `update()` after passing.

---

## 6. Combined Effect of the Adaptive Strategies

### Typical scenario 1: Normal driving

```
High odometry accuracy, occasional Limelight updates
→ Hub IMU angular velocity/acceleration steady → qBoost_x/y/θ ≈ 1.0
→ Q = Q_base (normal), R scaled from stdDev
→ Filter relies mostly on odometry prediction, vision only applies slow corrections
```

### Typical scenario 2: Robot collision (D2)

```
Hub IMU detects roll/pitch angular acceleration >> 5 rad/s² or yaw angular acceleration >> 4 rad/s²
→ qBoost on the affected axes jumps to 3-10x
→ Q on those axes grows, so the filter stops trusting odometry prediction
→ At the next vision update, the Kalman gain K on those axes is large
→ Pose quickly converges to the vision measurement
→ Per-axis independence: only impacted axes boost Q, others stay normal
```

### Typical scenario 3: Slope change (D3 only)

```
While driving uphill or downhill, roll/pitch angular velocity >> 1 rad/s
→ D3 angular-velocity detector fires → qBoost_x/y rises to 2-10x
→ Q grows moderately, accounting for odometry unreliability during slope transitions
→ Once the slope stabilizes (angular velocity drops), qBoost decays quickly → back to normal Q
```

### Typical scenario 4: Partial Limelight occlusion (single tag, long distance)

```
MT1 is still valid, but per-axis stdDev is uneven
→ Example: stdX=0.08m (3.1in), stdY=0.12m (4.7in), stdYaw=8°
→ R_x ≈ 0.05, R_y ≈ 0.12, R_θ ≈ 0.17
→ Vision corrects x strongly but y and theta weakly
→ Avoids a single tag pulling every axis uniformly off course
```

### Typical scenario 5: Total vision loss

```
MT1 invalid for a long time (no tags or occluded)
→ Predict only, no update
→ Filter runs on odometry alone, covariance P grows over time
→ Once vision recovers (low stdDev), K automatically grows and corrects quickly
```

---

## 7. Unit Consistency

The filter state uniformly uses **inches + radians**; all sensor data is converted before entering the filter:

| Input | Raw unit | Conversion | Unit at filter input |
|---|---|---|---|
| Pinpoint velocity (vx, vy) | in/s | No conversion | in/s |
| Pinpoint angular velocity (ω) | rad/s | No conversion | rad/s |
| Limelight position (x, y) | meters (m) | × 39.37 | inches (in) |
| Limelight heading (θ) | degrees (°) | × π/180 | radians (rad) |
| MT1 stdDev position (x, y) | meters (m) | × 39.37 | inches (in) |
| MT1 stdDev angle (yaw) | degrees (°) | × π/180 | radians (rad) |
| **Filter output pose** | — | — | **inches + radians** |

> **Independent angle thresholds**: `mapStdToR` compares position stdDev against `STD_LOW_INCH`/`STD_HIGH_INCH`; `mapStdToRAngle` compares angle stdDev against `STD_LOW_ANGLE`/`STD_HIGH_ANGLE`, so radian values are never mistakenly compared against inch values.

Conversion constant:
```java
private static final double M_TO_INCH = 39.37007874;
```

---

## 8. API Data Flow

```
adaptQ(double dt)
  │
  ├─ D2 mode:
  │   ├─ hubImu.getRobotAngularVelocity() → rollRate(xRotationRate), pitchRate(yRotationRate)
  │   │   → rollAccel, pitchAccel → rotate into field frame (Rz(θ))
  │   │   → fieldX = rollAccel·cosH - pitchAccel·sinH
  │   │   → fieldY = rollAccel·sinH + pitchAccel·cosH
  │   │   → cross mapping: qBoostX ← |fieldY|, qBoostY ← |fieldX|
  │   └─ hubImu.getRobotAngularVelocity() → yawRate → |α_yaw| → qBoostTheta
  │
  ├─ D3 mode:
  │   ├─ hubImu.getRobotAngularVelocity() → rollRate, pitchRate
  │   │   ├─ fieldVelX = rollRate·cosH - pitchRate·sinH
  │   │   ├─ fieldVelY = rollRate·sinH + pitchRate·cosH
  │   │   → cross mapping: qBoostX ← |fieldVelY|, qBoostY ← |fieldVelX|
  │   └─ hubImu.getRobotAngularVelocity() → yawRate
  │       └─ |α_yaw| → updateBoost → qBoostTheta  (yaw rate excluded)
  │
  └─→ build diagonal 3x3 SimpleMatrix Q (in²/s)  →  return Q

adaptR()
  │
  ├─ mt1.getStdDevs()  →  stdX_m, stdY_m, stdYaw_deg
  ├─ stdX_m × 39.37 → stdX_inch → mapStdToR  → × distFactor × tagFactor → rX
  ├─ stdY_m × 39.37 → stdY_inch → mapStdToR  → × distFactor × tagFactor → rY
  └─ Math.toRadians(stdYaw_deg) → mapStdToRAngle → rTheta (angle, unscaled)
  │
  └─→ build diagonal 3x3 SimpleMatrix R (in²)  →  return R

Vision update gate (mt1.isValid() && mt1.isHiveEstimated()):
  ├─ isHiveEstimated() == false → HIVE tilt has no solution or is rejected this frame
  │    (no tags / no solution / root selection above CellUpAngle / pose cross-check failed / pivot height uncalibrated)
  │    → mt1.getPose() falls back to the uncorrected contaminated pose, whole frame discarded
  └─ Pass → adaptR() → setR(R) → gateVision(x, y, θ, GATE_THRESHOLD) must pass before update(x, y, θ, timestamp)

Baseline (EKFLocalizer / UKFLocalizer):
  No Mahalanobis gating, but still requires isValid() && isHiveEstimated()

EKF/UKF:
  setQ(SimpleMatrix Q)   — accepts a complete Q matrix directly
  setR(SimpleMatrix R)   — accepts a complete R matrix directly
  setQ(double, double, double)  — scalar interface (kept)
  setR(double, double, double)  — scalar interface (kept)
```

---

## 9. Timestamp Management and Data Consistency

### 9.1 Two time bases

| Source | Time base | Purpose |
|---|---|---|
| `System.nanoTime()` | Java monotonic clock | dt computation for predict |
| `LLResult.getTimestamp()` | Limelight hardware timestamp | measurement time for update |

### 9.2 Stale-data protection

The filter maintains `lastUpdateTime` internally and rejects any measurement whose timestamp is less than or equal to the last update, preventing network latency or frame-rate glitches from rolling data backward.

### 9.3 First-predict handling

On the first `predict` call, only the timestamp is recorded and no state is propagated, avoiding a divide-by-zero from dt == 0.

---

## 10. References

- Kou & Haggenmiller (2023), "Extended Kalman Filter State Estimation for Autonomous Competition Robots"
- Wan & van der Merwe (2001), "The Unscented Kalman Filter" — classic UKF paper
- Julier & Uhlmann (1997), "A New Extension of the Kalman Filter to Nonlinear Systems" — theoretical foundation of the unscented transform
- Limelight MegaTag1 docs: https://docs.limelightvision.io/docs/docs-megatag
- GoBilda Pinpoint docs: https://docs.gobilda.com/pinpoint-odometer
- BHI260IMU docs: https://www.bosch-sensortec.com/products/smart-sensors/bhi260/
- EJML (Efficient Java Matrix Library): https://ejml.org