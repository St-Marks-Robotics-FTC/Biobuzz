# ReplayBuffer Functionality and Integration Guide

This document explains how [ReplayBuffer.java](./ReplayBuffer.java) works and gives the complete set of changes for integrating "vision timestamp rollback replay" into a **legacy localization pipeline** (`Processors/FusionLocalizer` + `utility/filter/EKF` + `utility/filter/UKF`).

> Applicable scenario: another project already has a working `FusionLocalizer` (EKF/UKF fused localization) but **no** `ReplayBuffer`. After applying the changes below, it gains rollback replay aligned with vision latency.
>
> If you only need to get it running and do not need replay yet, do just [§5.1](#51-step-1-copy-replaybufferjava) and the constructor part where `allowReplay` defaults to `false`; when `allowReplay=false` the replay logic takes no part in the computation and behavior is bit-for-bit identical to the legacy version.

---

## 1. Background and Goals

### 1.1 The Problem: Vision Pipeline Latency

Limelight has an **end-to-end latency** from exposure until the FTC side receives `LLResult` (exposure + transport + on-board solve), typically on the order of tens of milliseconds. If this is not handled:

- `predict()` advances the state using "the current instant";
- `update()` instead fuses a vision observation "captured tens of milliseconds ago" as if it were current.

The two time bases disagree, so vision corrections are systematically "shifted backwards in time". At **constant or low speed** this is negligible, but during **acceleration/deceleration or sharp turns** it shows up as a clear lag error (the pose is "dragged behind" by vision).

### 1.2 Approach

The idea is borrowed from WPILib `PoseEstimator` -- "align a timestamped observation to the instant it actually occurred, then fuse" -- but this implementation targets **recursive filters (EKF/UKF)** and instead does:

> **Roll the filter state back -> update at the observation instant -> replay the odometry predictions that followed to return to the current instant**

This differs from `PoseEstimator` (which interpolates a pose buffer and then applies a uniform transform); the two cannot be derived from one another.

### 1.3 Files Involved

| File | Role | Required? |
|---|---|---|
| [`utility/filter/ReplayBuffer.java`](./ReplayBuffer.java) | Rollback replay buffer | **New** |
| `utility/filter/EKF/EKF.java` | Add snapshot recording + rollback replay | **Modified** |
| `utility/filter/UKF/UKF.java` | Same as above | **Modified** |
| `Processors/FusionLocalizer/EKFLocalizer.java` | Pass through `allowReplay` + choose the right timestamp | **Modified** |
| `Processors/FusionLocalizer/UKFLocalizer.java` | Same as above | **Modified** |
| `Processors/FusionLocalizer/AdaptiveEKFLocalizer.java` | Same as above | **Modified** |
| `Processors/FusionLocalizer/AdaptiveUKFLocalizer.java` | Same as above | **Modified** |
| `Processors/VisionLocalizer/MT1Localizer.java` | Provide a capture instant on the `System.nanoTime()` base | **Modified (dependency)** |

> Dependency direction: `EKF`/`UKF` (packages `...utility.filter.EKF` / `...utility.filter.UKF`) `import ...utility.filter.ReplayBuffer`. The four `FusionLocalizer` classes do **not** depend on `ReplayBuffer` directly; they only call `ekf.isReplayEnabled()` and choose the timestamp.

---

## 2. ReplayBuffer in Detail

### 2.1 What It Stores

After every `predict()` it records a "state snapshot" containing:

| Field | Meaning | Purpose |
|---|---|---|
| `t` | Snapshot timestamp (seconds, `System.nanoTime()` base) | Time index for `floor()`/`after()` |
| `x` | Copy of the state `[x, y, θ]` | Restore on rollback |
| `P` | Copy of the 3×3 state covariance | Restore on rollback |
| `Q` | Copy of the 3×3 process noise **used to propagate to this snapshot** | Reused by historical value during replay (see §2.5) |
| `vx, vy, omega` | **Control input** used to propagate the previous snapshot to this one | Velocity input for the replay segment |

> Key semantics: in `Snapshot(t_k, x_k, P_k, Q_k, u_k)`, `u_k` is "the input used for the segment `t_{k-1}→t_k`", while `Q_k` is "the process noise used for the segment `t_{k-1}→t_k`". So after `floor()` finds the base, it needs the `u`/`Q` of the first snapshot after it in order to advance to the observation instant.

### 2.2 Internal Structure

```java
private final double duration;                          // retention window (default 1.5 s)
private final ArrayDeque<Snapshot> samples = new ArrayDeque<>();
```

### 2.3 Key Invariants and Semantics

| Method | Semantics | Boundary behavior |
|---|---|---|
| `add(t, x, P, Q, vx, vy, omega)` | Append a snapshot | **Timestamps must be strictly increasing**: when `t <= tail.t` the sample is dropped outright (protects the monotonicity assumption behind `floor/after`) |
| Eviction | Keep the last `duration` seconds | Pop the head while `samples.size() > 2 && head.t < t - duration`; **always keep at least 2** snapshots so rollback is possible |
| `floor(t)` | Return the **newest** snapshot with `t_s <= t` | If every snapshot is later than `t` → return `null` (observation precedes the buffer window) |
| `after(t)` | Return all snapshots with `t_s > t`, ascending | Returns an empty list if there are none |
| `clear()` | Clear everything | Called from `reset()` |
| `size()` / `isEmpty()` | Debugging/statistics | — |

### 2.4 Copy Semantics

- When a `Snapshot` is constructed, `x` is `clone()`d and `P`/`Q` are `copy()`d, **decoupling** them from the filter's internal matrices and preventing later reassignment or in-place mutation such as `P = A·P·Aᵀ+...` from polluting the history.
- `getX()/getP()/getQ()` likewise return copies.
- After a replay completes, `refresh(x, P)` **writes back** the snapshot's state and covariance (`Q` is not refreshed, because the Q of a historical segment is immutable), keeping the buffer consistent with the filter's current progress.

### 2.5 Why Replay Must Use the "Historical Q" Instead of the Current Q

Under the adaptive strategy `Q` jumps from frame to frame (IMU impact/slope detection → `qBoost` changes). If the **current** Q were used to replay the whole history:

- the noise level of "the newest frame" would be injected into the covariance of "some past interval";
- the replayed result would not match the propagation that actually happened → the rollback-replay does not close and a systematic bias accumulates.

Hence a snapshot must store the `Q` in effect at the time, and each replay segment uses its own segment's historical Q. Note that `R` is the noise of "this observation itself" and is independent of the time axis, so replay keeps using the `R` computed by `adaptR()` for the current frame.

---

## 3. Rollback Replay: Principle and Timing

### 3.1 Timing Within One Frame (replay enabled)

```
EKF/UKF side                                    Localizer side
────────────────────────────────────────────   ─────────────────────────────
predict(u_now, t_now)                           odom.update() → velocity
  ├─ propagate state/covariance
  └─ replayBuffer.add(t_now, x, P, Q, u_now)    mt1.update()
                                                └─ timestampNano = now - latency
update(z, t_obs)  ← t_obs = timestampNano       fetch this frame's HIVE-corrected pose
  ├─ base = floor(t_obs)                        (only isValid && isHiveEstimated)
  ├─ roll back to base
  ├─ advance to t_obs using tail[0].u / tail[0].Q
  ├─ applyVisionUpdate(z)   ← fuse at t_obs
  └─ for each snapshot in tail:
        applyMotion(s.u, s.t - prevT, s.Q)
        s.refresh(...)
```

### 3.2 `replayUpdate` Step by Step

1. `base = replayBuffer.floor(t_obs)`: find the newest snapshot no later than the observation instant.
   - `base == null` → the observation precedes the buffer window; **abandon** this replay (return `false`, do not update `lastUpdateTime`).
2. Gap guard: `t_obs - base.t > MAX_REPLAY_GAP (1.0 s)` → abandon (predictions stopped updating / clock anomaly) to avoid replaying an enormous `dt`.
3. Rollback: `state = base.x`, `P = base.P`.
4. Advance to the observation instant: use the input and historical Q of the **first** snapshot from `after(base.t)`, with `dt = t_obs - base.t`.
5. Perform one standard vision update `applyVisionUpdate(z)` at the observation instant.
6. Replay back to the current instant: for each snapshot from `after(base.t)`, call `applyMotion(u, dt, Q)` in order and `refresh` the write-back.

> Note: because `floor` returns "the newest snapshot with ≤ t_obs", every snapshot in `after(base.t)` has a timestamp **strictly greater** than `t_obs`, so `dt>0` in step 4 and `dt>0` in every segment of step 6 always hold.

### 3.3 Behavior When `allowReplay=false`

Fully equivalent to the legacy version: no buffer is created, no snapshots are recorded, and `update()` does `lastUpdateTime = timestamp; applyVisionUpdate(...)` without entering `replayUpdate`.

---

## 4. Integration Overview (change list)

| File | Change summary |
|---|---|
| `ReplayBuffer.java` | New (copy as-is; package `org.firstinspires.ftc.teamcode.utility.filter`) |
| `EKF.java` | Add `allowReplay`/`replayBuffer`/`MAX_REPLAY_GAP`; constructor overloads; extract `applyMotion` out of `predict`; extract `applyVisionUpdate` out of `update` and add the replay branch; add `replayUpdate`/`toColumn`/`isReplayEnabled`/`getReplayBufferSize`; `reset` clears the buffer |
| `UKF.java` | Same as `EKF.java` (`applyMotion` implemented with the unscented transform; same signature) |
| `EKFLocalizer.java` | Add `allowReplay` to the constructor; change the `update` timestamp to `ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()` |
| `UKFLocalizer.java` | Same as above |
| `AdaptiveEKFLocalizer.java` | Same as above (including the full 7-argument constructor) |
| `AdaptiveUKFLocalizer.java` | Same as above |
| `MT1Localizer.java` | Add `getTimestampNanoBase()` (converts the capture instant onto the `System.nanoTime()` base) and `isNewFrame` deduplication |

---

## 5. Detailed Steps

### 5.1 Step 1: Copy `ReplayBuffer.java`

Copy [ReplayBuffer.java](./ReplayBuffer.java) verbatim into the target project:

```
<project>\TeamCode\src\main\java\org\firstinspires\ftc\teamcode\utility\filter\ReplayBuffer.java
```

Requirements:
- **Package name unchanged**: `package org.firstinspires.ftc.teamcode.utility.filter;`
- Depends on `org.ejml.simple.SimpleMatrix` (RoadRunner already bundles EJML, so no extra dependency is needed).

### 5.2 Step 2: Modify `EKF.java`

> Location: `utility/filter/EKF/EKF.java`. If the target project uses different class or package names, migrate it with the same logic.

#### 5.2.1 New imports and fields

```java
import java.util.List;

import org.firstinspires.ftc.teamcode.utility.filter.ReplayBuffer;

// ... existing fields ...

/** Timestamp of the previous update, used to reject stale observations (keep it if the legacy version already has one) */
private Double lastUpdateTime = null;

/** Whether vision timestamp rollback replay is enabled */
private final boolean allowReplay;

/** Rollback replay buffer (created only when allowReplay is true, otherwise null) */
private final ReplayBuffer replayBuffer;

/** Maximum gap (seconds) allowed between the rollback base and the observation instant; beyond it the rollback is abandoned */
private static final double MAX_REPLAY_GAP = 1.0;
```

#### 5.2.2 Constructor overloads

```java
public EKF(double initialX, double initialY, double initialTheta) {
    this(initialX, initialY, initialTheta, false);
}

public EKF(double initialX, double initialY, double initialTheta, boolean allowReplay) {
    // ... the existing state / P / Q / R initialization stays unchanged ...

    this.allowReplay = allowReplay;
    this.replayBuffer = allowReplay ? new ReplayBuffer() : null;
}
```

#### 5.2.3 `predict`: record snapshots and move the propagation logic into `applyMotion`

```java
public void predict(double vx, double vy, double omega, double timestamp) {
    // First call: only record the timestamp, do not predict
    if (lastPredictTime == null) {
        lastPredictTime = timestamp;
        if (allowReplay) {
            replayBuffer.add(timestamp, getPose(), P, Q, 0, 0, 0);
        }
        return;
    }

    double dt = timestamp - lastPredictTime;
    lastPredictTime = timestamp;

    // Guard against an abnormal dt
    if (dt <= 0 || dt > 1.0) {
        return;
    }

    applyMotion(vx, vy, omega, dt, Q);

    // Record a snapshot (together with the historical Q and input used for this segment)
    if (allowReplay) {
        replayBuffer.add(timestamp, getPose(), P, Q, vx, vy, omega);
    }
}
```

Extract "state propagation + Jacobian + covariance propagation" from the original `predict` into a private method with **one extra `Qused` parameter**:

```java
private void applyMotion(double vx, double vy, double omega, double dt, SimpleMatrix Qused) {
    double x = state.get(0, 0);
    double y = state.get(1, 0);
    double theta = state.get(2, 0);

    double cosTheta = Math.cos(theta);
    double sinTheta = Math.sin(theta);

    double xNew = x + dt * (vx * cosTheta - vy * sinTheta);
    double yNew = y + dt * (vx * sinTheta + vy * cosTheta);
    double thetaNew = theta + dt * omega;

    SimpleMatrix A = new SimpleMatrix(new double[][]{
            {1, 0, dt * (-vx * sinTheta - vy * cosTheta)},
            {0, 1, dt * ( vx * cosTheta - vy * sinTheta)},
            {0, 0, 1                                     }
    });

    // A regular predict passes the current Q; replay passes the historical Q recorded for that segment
    SimpleMatrix Qdt = Qused.scale(dt);
    P = A.mult(P).mult(A.transpose()).plus(Qdt);

    state = new SimpleMatrix(new double[][]{
            {xNew},
            {yNew},
            {normalizeAngle(thetaNew)}
    });
}
```

#### 5.2.4 `update`: add the replay branch and move the update body into `applyVisionUpdate`

```java
public void update(double xMeas, double yMeas, double thetaMeas, double timestamp) {
    // Reject stale observations (kept whether or not replay is enabled)
    if (lastUpdateTime != null && timestamp <= lastUpdateTime) {
        return;
    }

    if (allowReplay) {
        if (replayUpdate(xMeas, yMeas, thetaMeas, timestamp)) {
            lastUpdateTime = timestamp;
        }
        return;
    }

    lastUpdateTime = timestamp;
    applyVisionUpdate(xMeas, yMeas, thetaMeas);
}
```

`applyVisionUpdate` is exactly the body of the legacy `update` -- "innovation → S → K → state → covariance" -- moved over unchanged:

```java
private void applyVisionUpdate(double xMeas, double yMeas, double thetaMeas) {
    SimpleMatrix z = new SimpleMatrix(new double[][]{{xMeas}, {yMeas}, {thetaMeas}});

    SimpleMatrix yInnov = z.minus(state);
    yInnov.set(2, 0, normalizeAngle(yInnov.get(2, 0)));

    SimpleMatrix S = H.mult(P).mult(H.transpose()).plus(R);
    SimpleMatrix K = P.mult(H.transpose()).mult(safeInvert(S));

    state = state.plus(K.mult(yInnov));
    state.set(2, 0, normalizeAngle(state.get(2, 0)));

    SimpleMatrix I = SimpleMatrix.identity(3);
    P = I.minus(K.mult(H)).mult(P);
}
```

Add `replayUpdate` and `toColumn`:

```java
private boolean replayUpdate(double xMeas, double yMeas, double thetaMeas, double timestamp) {
    ReplayBuffer.Snapshot base = replayBuffer.floor(timestamp);
    if (base == null) {
        return false;               // observation precedes the buffer window; cannot roll back
    }
    if (timestamp - base.t > MAX_REPLAY_GAP) {
        return false;               // gap too large; abandon the rollback
    }

    List<ReplayBuffer.Snapshot> tail = replayBuffer.after(base.t);   // t strictly > observation instant

    // 1. Roll back to the newest snapshot at or before the observation instant
    state = toColumn(base.getX());
    P = base.getP();

    // 2. Advance from that snapshot to the observation instant (using the input and historical Q of the first segment after it)
    if (!tail.isEmpty()) {
        ReplayBuffer.Snapshot first = tail.get(0);
        applyMotion(first.vx, first.vy, first.omega, timestamp - base.t, first.getQ());
    }

    // 3. Apply the update at the observation instant (R stays the R of the current observation itself)
    applyVisionUpdate(xMeas, yMeas, thetaMeas);

    // 4. Replay all later predictions back to the current instant and refresh the snapshots
    double prevT = timestamp;
    for (ReplayBuffer.Snapshot s : tail) {
        applyMotion(s.vx, s.vy, s.omega, s.t - prevT, s.getQ());
        prevT = s.t;
        s.refresh(getPose(), P);
    }
    return true;
}

private SimpleMatrix toColumn(double[] v) {
    return new SimpleMatrix(new double[][]{{v[0]}, {v[1]}, {v[2]}});
}
```

#### 5.2.5 `reset`: clear the buffer + reset timestamps

```java
public void reset(double x, double y, double theta) {
    // ... the existing state / P reset stays unchanged ...
    lastPredictTime = null;
    lastUpdateTime = null;
    if (replayBuffer != null) {
        replayBuffer.clear();
    }
}
```

#### 5.2.6 New accessors

```java
/** @return whether vision timestamp rollback replay is enabled */
public boolean isReplayEnabled() {
    return allowReplay;
}

/** @return number of snapshots in the replay buffer (0 when replay is disabled) */
public int getReplayBufferSize() {
    return replayBuffer == null ? 0 : replayBuffer.size();
}
```

### 5.3 Step 3: Modify `UKF.java`

The changes correspond **one for one** with `EKF.java`; only the algorithm inside `applyMotion` differs:

- `applyMotion(vx, vy, omega, dt, Qused)`: propagate with the sigma-point unscented transform (generate sigma points → propagate each point → circular mean → weighted covariance + `Qused.scale(dt)`).
- `update`/`replayUpdate`/`applyVisionUpdate`/`toColumn`/`reset`/constructors/accessors: the structure is identical to EKF; just use UKF's existing implementations of `H`, `safeInvert`, and `normalizeAngle`.

> Reference: `predict` / `applyMotion` / `update` / `applyVisionUpdate` / `replayUpdate` in [UKF.java](../UKF/UKF.java).

### 5.4 Step 4: Modify the Four `FusionLocalizer` Localizers

Apply **two kinds** of change to `EKFLocalizer`, `UKFLocalizer`, `AdaptiveEKFLocalizer`, and `AdaptiveUKFLocalizer`:

#### 5.4.1 Pass `allowReplay` Through the Constructors

- Non-adaptive (`EKFLocalizer` / `UKFLocalizer`): add a `boolean allowReplay` overload to each D2/D3 constructor and pass it through to the filter:

```java
public EKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight, Pose2d initialPose,
                    boolean allowReplay) {
    this.ekf = new EKF(initialPose.position.x, initialPose.position.y,
                       initialPose.heading.toDouble(), allowReplay);   // ← the only change
    ekf.setQ(QbasePos, QbasePos, QbaseAngle);
    ekf.setR(RbasePos, RbasePos, RbaseAngle);
    this.odom = new PinpointLocalizer(hardwareMap, 0.001999, initialPose);
    this.mt1 = new MT1Localizer(limelight);
    this.useD3 = false;
}
```

- Adaptive (`AdaptiveEKFLocalizer` / `AdaptiveUKFLocalizer`): append `allowReplay` after the existing `useD3` and `teamColor`:

```java
public AdaptiveEKFLocalizer(HardwareMap hardwareMap, Limelight3A limelight,
                            String imuDeviceName, Pose2d initialPose, boolean useD3,
                            TeamColor teamColor, boolean allowReplay) {
    this.ekf = new EKF(initialPose.position.x, initialPose.position.y,
                       initialPose.heading.toDouble(), allowReplay);   // ← the only change
    // ... odom / mt1 / hubImu initialization stays unchanged ...
}
```

> Keep the old constructor overloads and delegate with `allowReplay=false` internally so existing call sites need no changes.

#### 5.4.2 Choose the Right Timestamp Base in `update()`

This is the most critical part. **Replay requires the observation timestamp to share the base of the `predict` snapshots (`System.nanoTime()`).**

- Replay disabled: keep using the Limelight hardware timestamp (used only for `lastUpdateTime` deduplication and not involved in rollback).
- Replay enabled: switch to `mt1.getTimestampNanoBase()`.

In all four localizers, change the vision-update block in `update()` to:

```java
if (mt1.isValid() && mt1.isHiveEstimated()) {
    ekf.setR(adaptR());                        // ukf.setR(...) in UKFLocalizer
    Pose2d visionPose = mt1.getPose();
    if (ekf.gateVision(                       // omit this if when UKFLocalizer has no gating
            visionPose.position.x,
            visionPose.position.y,
            visionPose.heading.toDouble(),
            GATE_THRESHOLD)) {
        ekf.update(
                visionPose.position.x,
                visionPose.position.y,
                visionPose.heading.toDouble(),
                // Rollback replay: same base as the predict snapshots; otherwise keep the Limelight hardware timestamp
                ekf.isReplayEnabled() ? mt1.getTimestampNanoBase() : mt1.getTimestamp()
        );
    }
}
```

> Notes:
> - `EKFLocalizer` / `UKFLocalizer` (the control group) do **not** apply Mahalanobis gating and call `update` directly, but they **also** need the timestamp-selection change.
> - `AdaptiveUKFLocalizer` / `UKFLocalizer` use `ukf` instead of `ekf`.

### 5.5 Step 5: Modify `MT1Localizer` (Timestamp Base Dependency)

If the legacy `MT1Localizer` only has `getTimestamp()` (the Limelight hardware clock), you must add a capture instant on the `System.nanoTime()` base; otherwise replay cannot align with the prediction snapshots.

#### 5.5.1 New Fields

```java
/** Timestamp (seconds) of this frame's capture instant, already converted to the System.nanoTime() base */
private double timestampNano;

/** Limelight hardware timestamp (seconds) of the previous physical frame, used to tell whether a frame is new; initialized to NaN so the first frame counts as new */
private double lastFrameTimestamp = Double.NaN;
```

#### 5.5.2 Computation in `update()`

```java
double frameTimestamp = latestResult.getTimestamp();
captureLatency = latestResult.getCaptureLatency();

// Treat it as a new physical frame only when the hardware timestamp changes: when the frame rate is below the loop rate, getLatestResult() returns the same frame repeatedly
boolean isNewFrame = frameTimestamp != lastFrameTimestamp;
lastFrameTimestamp = frameTimestamp;
timestamp = frameTimestamp;

// Convert to the System.nanoTime() base: the local instant the result arrived minus the capture latency
// Refresh only on a new frame; repeated reads of the same frame stay constant so the same observation is not fused or rolled back twice
if (isNewFrame) {
    timestampNano = System.nanoTime() / 1e9 - captureLatency / 1000.0;
}
```

#### 5.5.3 New Accessor

```java
/** @return this frame's capture instant (seconds), already converted to the System.nanoTime() base, for use by rollback replay */
public double getTimestampNanoBase() {
    return timestampNano;
}
```

> The two clocks cannot be subtracted directly: the Limelight hardware timestamp and `System.nanoTime()` are on different bases. **Only `timestampNano` can be compared with the snapshot timestamps of `predict` inside EKF/UKF.**

---

## 6. Verification and Debugging

1. **Replay-off control**: construct two localizers from the same OpMode, one with `allowReplay=false` and one with `true`; their poses should agree while vision is valid, and while maneuvering (accel/decel/sharp turns) the `true` one should lag less.
2. **Buffer size**: print `localizer.getEKF().getReplayBufferSize()` (`getUKF()` for UKF). With a steady loop it should be close to "`1.5 s ÷ frame period`", e.g. ≈ 75 at 50 Hz. If it stays at 0/1 for a long time, `predict` is not advancing or the timestamp is not increasing.
3. **Does the rollback hit?**: if an observation precedes the buffer window or the gap is > `MAX_REPLAY_GAP`, `replayUpdate` returns `false` and does **not** update `lastUpdateTime`. Add temporary logging to confirm frames are not being discarded permanently.
4. **Timestamp monotonicity**: `getTimestampNanoBase()` must increase with each new physical frame; if it is constant or goes backwards, deduplication (`timestamp <= lastUpdateTime`) silently drops vision, so check the `isNewFrame` criterion and the firmware timestamp.

---

## 7. Common Pitfalls

1. **Same base is mandatory**: if `predict` uses `System.nanoTime()`, observations must use `getTimestampNanoBase()`; otherwise `floor()` compares a hardware timestamp against `System.nanoTime()`, and rollback either never works or triggers a huge `dt`.
2. **Do not share the hardware timestamp for rollback**: the hardware timestamp serves only `lastUpdateTime` deduplication. Comparing across clocks is the classic mistake.
3. **Fuse each physical frame once**: `timestampNano` is refreshed only on `isNewFrame`; together with the filter's `lastUpdateTime` deduplication this prevents the same observation from being counted twice when the Limelight frame rate is below the loop rate.
4. **When to record `Q`**: `predict` must record the snapshot **after** propagating (the `getPose()/P` after `applyMotion`), and it must record the `Q`/`u` "used for this segment"; otherwise `after()` replay is off by one segment.
5. **`R` is not per-snapshot**: replay keeps the current frame's `R`; do not store it in the snapshot as well.
6. **Threading model**: `ReplayBuffer` is not thread-safe and may only be accessed from the single OpMode loop thread; do not call `predict/update` from other threads.
7. **Memory**: each snapshot holds a few copies of 3×3 matrices; 1.5 s @ 100 Hz is ≈ 150 snapshots, a negligible amount.
8. **A failed rollback is not an error**: an observation from before the buffer window is a historical observation that cannot be aligned, and dropping it is correct; that frame's vision is skipped while `predict` proceeds as usual.

---

## 8. How to Disable Replay (Fallback)

- Simply omit `allowReplay` from every constructor (or pass `false`); then:
  - `EKF/UKF` create no buffer, `predict` records nothing, and `update` takes the non-replay branch;
  - the observation timestamp falls back to `mt1.getTimestamp()`;
  - runtime cost and behavior match the legacy version.

You can also remove every reference to `ReplayBuffer` to restore the legacy version, but since all changes are **incremental and off by default**, no code rollback is needed.

---

## Appendix: One-Sentence Summary

> `ReplayBuffer` stores a snapshot of "state/covariance/Q in use/input" after every `predict`; when a delayed vision observation arrives, it first rolls back to the observation instant to fuse, then replays the subsequent odometry predictions back to the current instant, eliminating the lag caused by vision pipeline delay. Integration = copy 1 new file + give EKF/UKF "snapshot recording + rollback replay" + pass `allowReplay` through the 4 localizers and select the `timestampNano` base in `update()` according to `isReplayEnabled()` + add `getTimestampNanoBase()` to `MT1Localizer`.
