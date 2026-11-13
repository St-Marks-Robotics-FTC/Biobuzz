---
name: pedropathing-v3-paths
description: "Curved bezier paths, heading interpolation, per-path speed limits, or spin-in-place in BozoAuto with Pedro Pathing v3; no follower.manual."
---

Verified via `javap` on `~/.gradle/caches/modules-2/files-2.1/com.pedropathing/core/3.0.x/*/core-3.0.x.jar` (class `com.pedropathing.api.Paths`, already `import static com.pedropathing.api.Paths.*` in BozoAuto).

## Curve API

- `curve(Pose...)` = bezier: first = start, last = end, everything between = control points (pulled toward, never driven through). Also `curve(Vector2D...)`, `through(Pose...)` (passes through), `line(Pose, Pose)`.
- Heading methods on `Path`: `.linear(startPose, endPose)`, `.constant(..)`, `.tangent()`, `.reverseTangent()`, `.facingPoint(..)`, `.heading(Interpolator)`, `.with(Modifier...)`.
- Control point heading is ignored; only x/y matter.
- `plainLine(a, b)` = `line(a,b).linear(a,b)`. Convert to curve: `curve(a, ctrl, b).linear(a, b)` (keeps heading).
- Red is Blue rotated 180° about field centre (72,72): x -> 144 - x AND y -> 144 - y, heading +180. Derive Red control points this way; never copy Blue values unmirrored.

## Heading interpolation

- User wants behavior built as a **Path**, never `follower.manual(...)`. Per-path overrides via `.with(Constants.foresightConfig.<var>.at(x))` are fine (https://pedropathing.com/docs/pathing/guide/modifiers); global `Constants` edits are not.
- `Interpolator.linear(a, b)` uses the **short arc** (`Angle.error` = normalizeSigned(b-a)); 330° and -30° are equivalent. `longLinear` exists for the long way.
- Hold-then-turn on one line: `line(a,b).heading(Interpolator.piecewise().until(0.7, Interpolator.constant(poseA)).until(1.0, Interpolator.linear(poseA, poseB)))`. `until(t, ...)` needs strictly increasing t ≤ 1. Also `path.linear(startH, endH, fraction)`.
- `Paths.line` throws on zero length — a pure rotate-in-place path is impossible; the turn must overlap translation.
- `isBusy()` includes a heading condition — the path isn't "done" until heading is within tolerance.
- Mirror across x=72 for Red: heading h -> 180 - h.
- Biobuzz: approach heading = `intermediatePose.heading()`; final = `refuelPose.heading()`; `BozoAuto.REFUEL_ROTATE_START` = 0.7.

## Speed limits

The user has rejected EVERY speed-limiting attempt in BozoAuto (Constants edits, manual-drive helpers + new Tunables — all reverted). Do not add helpers, new Tunables, or per-loop limiters unprompted; keep any change to a few lines and ask first even when the request names a mechanism.

- `Follower` has no `setMaxPower`: `follow`, `hold`, `manual(DrivePowers)`, `manual(fwd, strafe, turn)`, `isBusy`, `remainingDistance()`, `pose()`. `follow(Path)` resets mode to FOLLOW (manual->path is clean).
- Limits live in shared static `Constants.foresightConfig`: `maxPathSpeed`, `maxVelocityConstraint` are `ConfigVar<Double>`, default `Constraint.NONE` (Infinity), validated positive (0 rejected). Persists across OpModes unless reset — prefer per-path `.with(...at(x))`.
- `ManualDrive.fieldCentric(fwd, strafe, turn, heading)` only rotates forward/strafe; BozoTeleOp uses it with `follower.manual(powers)`.

## Spin in place

`follower.turnTo()` no longer exists. Teleop pattern: `follower.hold(new Pose(x, y, targetHeading))`, exit when `|pose.heading() - target| < Tunables.shootHeadingMargin`.

Multi-rotation spin (e.g. TEMP_FAILED): single `hold` can't express >180° — use 120° steps (3 rotations = 9 steps). Anchor each target to the heading at spin start (`start + (n+1)*step`), NOT measured heading. Wrap target with `atan2(sin, cos)` and compare with wrap-aware error (mod 2π into [-π, π]). Keep x/y from spin-start pose. Entering state: `shutOff()`, reset counter, capture start pose, hold first target, `setState(...)`; each loop: within margin → count++, next hold or `setState(DONE)`.

## Verify

`./gradlew :TeamCode:compileDebugJavaWithJavac --offline -q`. Throwaway javap smoke: unzip core jar to /tmp, `javac -cp` a small main, call `path.heading(t)`.
