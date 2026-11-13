---
name: ftc-auto-shooting-state-machine
description: "Porting/adapting shooting sequences into BozoAuto.java (flywheel spin-up, margin gate, transfer/intake feed timing); not teleop aiming."
---

## Context

Autonomous OpModes extend `TeamCode/.../auto/BozoAuto.java`, a Pedro Pathing state-machine `OpMode`. This robot is simpler than the upstream `St-Marks-Robotics-FTC/Decode` reference repo: only `Robot.flywheel` (`subsys/Flywheel.java`), `Robot.intake` (`subsys/Intake.java`), `Robot.transfer` (`subsys/Transfer.java`) — no turret, no vision artifact pattern, no `LaunchSetpoints`. Steal the Decode `BozoAuto.java` spin-up/margin-gate/feed-timer *idea*, not its literal states.

## Pattern

1. `robot.flywheel.setRPM(Tunables.shootRPM)` as soon as you start driving toward the shoot pose — spin up during travel, not after arriving.
2. Add `Flywheel.isWithinMargin()`: `targetRPM > 0 && Math.abs(getRPM() - targetRPM) <= Tunables.flywheelRPMMargin`. Guard `targetRPM == 0` so idle never reports "within margin".
3. At shoot pose (`!follower.isBusy()`), wait in a dedicated `SPIN_UP` state until `isWithinMargin()` — don't feed early.
4. On margin: `robot.transfer.open()` + `robot.intake.forward()`, transition to `FEED`, gate on `stateTimer.get(TimeUnit.MILLISECONDS) >= Tunables.feedDurationMillis` (no ball-count sensor exists — timing is the only gate; don't invent one).
5. After feed timeout: `robot.transfer.close()`, `robot.intake.off()`, `robot.flywheel.setRPM(0)`, increment `shotsCompleted`, branch to refuel leg or end pose via `TOTAL_SHOTS`.
6. Refuel leg: drive to refuel pose, intake forward for `Tunables.refuelDurationMillis`, drive back, re-spin flywheel on the way.
7. `Robot` has no aggregate `update()` — call `robot.flywheel.update()` explicitly every `loop()` (missing this silently means the PIDF never applies power).

## Tunables

`Tunables.java`: `shootRPM`, `flywheelRPMMargin`, `feedDurationMillis`, `refuelDurationMillis` (all `public static`; class itself is `@Configurable`, no per-field annotation needed).

## Verify

`./gradlew :TeamCode:compileDebugJavaWithJavac -q` only confirms API/type correctness — no hardware here; flag on-robot timing/RPM tuning as untested.
