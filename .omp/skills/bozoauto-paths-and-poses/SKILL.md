---
name: bozoauto-paths-and-poses
description: "Adding a new pose, bezier control point, or path/state branch to BozoAuto (AutoConfig, BlueAuto, RedAuto); mirror rule, tipped-hive branch, compile check."
---

Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` — `AutoConfig.java`, `BlueAuto.java`, `RedAuto.java`, `BozoAuto.java` (thin: config -> paths -> route), `AutoOpMode.java` (generic lifecycle, 30 s cutoff, telemetry, handoff; partner-robot autos extend this), `core/` (pure-Java `Step`, `Sequencer`, `Steps` combinators; unit tested in `TeamCode/src/test`), `steps/` (`AutoContext`, `RobotSteps` = drive/shoot/collectFlower/refuel/park, `HiveView`), `paths/` (`AutoPaths`, `FlowerSite`, `PathHelpers`), `routes/HiveRoute.java` (the sequence + branches as data). There is no state enum any more. `BlueCloseAuto`/`RedCloseAuto` are empty subclasses, no change needed.

## Procedure for each new pose

1. **AutoConfig**: add public field + constructor param + assignment. Constructor is POSITIONAL and every param is a `Pose`, so wrong order compiles and silently swaps poses. Keep field order, constructor order, and `buildConfig()` order identical, in path order (start, shoot, refuel, tipped, flower 1, flower 2, park). Read `AutoConfig.java` first — the field set evolves; never assume it.
2. **BlueAuto and RedAuto**: add `public static Pose X` (`@Configurable`, tunable in Panels) and pass it in `buildConfig()` in the same order. Mark untuned values `DUMMY coords, replace with tuned values` once per group. Keep comments minimal: group headings only.
3. **AutoPaths + HiveRoute**: add a `public final Path` field in `paths/AutoPaths.java` and build it in its constructor (`curve(start, control..., end).linear(start, end)` or `line(a, b).linear(a, b)`; control heading is ignored; helpers in `PathHelpers`), then add it to the sequence in `routes/HiveRoute.java` with `s.drive(label, path)` / `s.travelShoot(label, path)` / `s.collectFlower(site, approachPath)` / `s.park(path)`; wrap decisions in `branch(label, hive::tagsVisible, whenTrue, whenFalse)`. Reused fragments are local variables so cases cannot drift. Update the `HiveRoute` class javadoc. Any new hardware action belongs in `RobotSteps` once, never inline in a route. `Steps.describe(route)` prints the full plan (both branch arms) at init in telemetry/logcat.
4. Multiple control points: Pedro 3.0 `Paths.curve(Pose...)` is varargs — extra control points go between start and end.

## Mirror rule

Blue/Red are related by POINT REFLECTION through field centre: `(x, y, θ) -> (144-x, 144-y, θ+180°)` (e.g. heading 90<->270, 180<->0, 210<->30). Red is the tuned source of truth; when mirroring, copy Red into Blue with this rule (positions AND headings). `tippedShootPose` is computed from shootPose so it already follows. Verify mechanically: regex all `Pose name = new Pose(x, y, Math.toRadians(h))` in both files and assert blue == (144-x, 144-y, (h+180)%360) for every name.

Tipped (scoring-side) shoot pose = untipped shootPose with same x and y -> 144 - y (mirror over y=72); heading flipped (270<->90).

## Tipped-hive branch

Lives in `HiveRoute` (`HiveView.tagsVisible()` = Limelight botpose tag count `> Tunables.tagThreshold`, one comparison for every branch; tags visible = hive NOT tipped at the first judgement, and at the CASE1/CASE2 checkpoints visible means CASE1A/CASE2B). Flower pickup is `RobotSteps.collectFlower`: intake on + arm down, approach, creep in (`flowerApproachSpeed`, `maxTravelToRefuelTime` timeout), shake, wait `flowerRefuelDurationMillis`, back out at `flowerIntakeSpeed` WITH the shake still running, arm up. `park` calls `shutOff()` first.

## API / prefs

- Pose API: `x()`, `y()`, `heading()` — NOT `getX()`/`getY()`.
- "In between X and Y" for multi-instance legs (two flowers): apply to all instances and state that assumption.
- Edits to Blue/Red left uncommitted in working tree unless asked.

## Verify

`cd <repo> && JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew -q :TeamCode:compileDebugJavaWithJavac > /tmp/build.log 2>&1; echo $?` (check the real Gradle exit code, not a grep pipe's). System default Java may be 27 which breaks the build ("Unsupported class file major version 71"): always force JDK 21. After merging, rerun without `--offline` if deps were bumped. Post-merge "cannot resolve method X in Robot" usually means the other branch renamed it (e.g. `Robot.updateLaunch()` -> `Robot.update()`); grep `Robot.java` and the TeleOp caller. No hardware here: say on-robot behavior is untested.
