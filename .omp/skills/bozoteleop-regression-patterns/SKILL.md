---
name: bozoteleop-regression-patterns
description: "Use when Biobuzz drivetrain stops moving or flywheel target RPM changes but actual RPM doesn't follow in BozoTeleOp.java; not general FTC teleop."
---

Scope: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/teleop/BozoTeleOp.java`. Two recurring regressions during edits to `loop()`.

## Bug 1: drivetrain dead — inverted slow-mode multiplier

`slowModeMultiplier` gates every drive axis (`forward`, `strafe`, `turn`). It must be **1.0 at rest**, reducing only while the slow-mode trigger is held. Writing `gamepad1.left_trigger * 0.3` is `0` at rest — silently zeroes all drive output; robot looks dead while telemetry/loop runs fine.

```java
double slowModeMultiplier = 1.0 - (gamepad1.left_trigger * 0.7); // 1.0 normally, down to 0.3 while held
```

If you see `left_trigger * <scale>` with no `1.0 -` in front, that's the bug.

## Bug 2: flywheel target changes, actual RPM never follows

`robot.flywheel.update()` runs the PID (`pidf.calc` + `motor.setPower`). If `update()` is only called inside button-press blocks (`bWasPressed`, `dpadUpWasPressed`, `dpadDownWasPressed`) alongside `setRPM(...)`, the PID computes power **once per keypress**. Symptom: target RPM telemetry updates per press, actual RPM stays flat — nothing re-runs the PID between presses.

Fix: strip `update()` out of the button blocks (leave only `setRPM`/`isRunning` mutation) and call `robot.flywheel.update();` unconditionally once per `loop()`, right after button handling and before drivetrain code. Matches `subsys/Flywheel.java`'s live-reload `update()` (re-reads `Tunables.flywheel{P,I,D,F}` every tick) — the caller must also invoke it every tick.

## Verify

`./gradlew :TeamCode:compileDebugJavaWithJavac -q`. No on-robot harness; report as bench-test recommendation — the compiler won't catch either regression.
