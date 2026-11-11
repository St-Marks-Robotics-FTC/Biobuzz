---
name: bozoauto-paths-and-poses
description: "Adding a new pose, bezier control point, or path/state branch to BozoAuto (AutoConfig, BlueAuto, RedAuto); mirror rule, tipped-hive branch, compile check."
---

Files: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/auto/` — `AutoConfig.java`, `BlueAuto.java`, `RedAuto.java`, `BozoAuto.java`. `BlueAuto`/`RedAuto` are the concrete `@Autonomous` OpModes (the old `*CloseAuto` subclasses were removed in b1a8a43); they MUST NOT be `abstract`. A Sinister/Panels `NullPointerException: lockOpModesWhile(...) must not be null` on init means the selected OpMode class could not be instantiated (e.g. `@Autonomous` on an `abstract` class).

## Procedure for each new pose

1. **AutoConfig**: add public field + constructor param + assignment. Constructor is POSITIONAL and every param is a `Pose`, so wrong order compiles and silently swaps poses. Keep field order, constructor order, and `buildConfig()` order identical, in path order (start, shoot, refuel, tipped, flower 1, flower 2, park). Read `AutoConfig.java` first — the field set evolves; never assume it.
2. **BlueAuto and RedAuto**: add `public static Pose X` (`@Configurable`, tunable in Panels) and pass it in `buildConfig()` in the same order. Mark untuned values `DUMMY coords, replace with tuned values` once per group. Keep comments minimal: group headings only.
3. **BozoAuto**: add `Path` field, build in `buildPaths()` (`curve(start, control..., end).linear(start, end)` or `line(a, b).linear(a, b)`; control heading is ignored), add `State` enum entries, add switch cases in `autoUpdate()`, call `follower.follow(path)` + `setState(...)` at the transition. Update the class javadoc sequence AND prose plus path-field comments (they go stale easily).
4. Multiple control points: Pedro 3.0 `Paths.curve(Pose...)` is varargs — extra control points go between start and end.

## Mirror rule

Blue/Red are related by POINT REFLECTION through field centre: `(x, y, θ) -> (144-x, 144-y, θ+180°)` (e.g. heading 90<->270, 180<->0, 210<->30). Red is the tuned source of truth; when mirroring, copy Red into Blue with this rule (positions AND headings). `tippedShootPose` is computed from shootPose so it already follows. Verify mechanically: regex all `Pose name = new Pose(x, y, Math.toRadians(h))` in both files and assert blue == (144-x, 144-y, (h+180)%360) for every name.

Tipped (scoring-side) shoot pose = untipped shootPose with same x and y -> 144 - y (mirror over y=72); heading flipped (270<->90).

## Tipped-hive branch

Lives in `TRAVEL_TO_SHOOT` (Limelight botpose tag count vs `Tunables.tagThreshold`); not tipped = TEMP_FAILED. Tipped sequence: TRAVEL_TO_TIPPED_SHOOT -> SPEED_UP_SECOND_SHOOT -> FEED_SECOND_SHOOT -> per flower [TRAVEL_TO_FLOWER(_2)_INTERMEDIATE -> TRAVEL_TO_FLOWER(_2) -> COLLECT_FLOWER(_2) -> TRAVEL_FROM_FLOWER(_2)_TO_INTERMEDIATE -> TRAVEL_TO_SHOOT_2/3 -> SPEED_UP_SHOOT_2/3 -> FEED_SHOOT_2/3] -> TRAVEL_TO_PARK. Flywheel stays on until FEED_SHOOT_3 (`shutOff()` only there). Flower legs: approach to intermediate at full speed (flower 1 line, flower 2 curve w/ 1 control pt), intermediate->flower line capped via `Constants.foresightConfig.maxPathSpeed.at(FLOWER_APPROACH_SPEED=0.3)`, then back out to intermediate, then line to tippedShootPose.

## API / prefs

- Pose API: `x()`, `y()`, `heading()` — NOT `getX()`/`getY()`.
- "In between X and Y" for multi-instance legs (two flowers): apply to all instances and state that assumption.
- Edits to Blue/Red left uncommitted in working tree unless asked.

## Verify

`cd <repo> && JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew -q :TeamCode:compileDebugJavaWithJavac > /tmp/build.log 2>&1; echo $?` (check the real Gradle exit code, not a grep pipe's). System default Java may be 27 which breaks the build ("Unsupported class file major version 71"): always force JDK 21. After merging, rerun without `--offline` if deps were bumped. Post-merge "cannot resolve method X in Robot" usually means the other branch renamed it (e.g. `Robot.updateLaunch()` -> `Robot.update()`); grep `Robot.java` and the TeleOp caller. No hardware here: say on-robot behavior is untested.
