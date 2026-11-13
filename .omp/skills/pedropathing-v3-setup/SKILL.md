---
name: pedropathing-v3-setup
description: "PedroPathing v3 config IllegalStateException, follower stub vs classic API, missing Ivy/Scheduler, or @Tuner procedures not in the OpMode list."
---

## Required vs default config fields

Symptom: `java.lang.IllegalStateException: Config variable has not been set` — e.g. running an AutoTune `@Tuner` procedure, or any path constructing `PinpointLocalizer`/`Mecanum`/`Foresight` from `Constants.java`.

Root cause: each field of `PinpointConfig`/`MecanumConfig`/`ForesightConfig` (`com.pedropathing.config.ConfigVar<T>`) is `ConfigVar.required(...)` (MUST `.set()` or it throws at read time) or `ConfigVar.of(default, ...)` (safe default). You cannot tell which from the field list — inspect constructor bytecode:

1. Find the actually-resolved jar (not a stale `-sources.jar`): check the `.pom` (e.g. `~/.gradle/caches/modules-2/files-2.1/com.pedropathing/revhub/3.0.0/*/revhub-3.0.0.pom`) for the transitively pulled `core` version.
2. `unzip -o` the resolved `.jar`/`.aar`'s `classes.jar` for the class (`com/pedropathing/algorithm/ForesightConfig.class`, `.../revhub/localizers/PinpointConfig.class`, `.../revhub/drivetrains/MecanumConfig.class`).
3. `javap -p -c -constants ClassName.class`: `putfield` preceded by `ConfigVar.required(...)` MUST be `.set()`; `ConfigVar.of(...)` has a default.

Confirmed for core 3.0.0 — `PinpointConfig` required: `name`, `xPodDirection`, `yPodDirection`, `xPodOffset`, `yPodOffset`. `MecanumConfig` required: all 4 motor names + directions. `ForesightConfig` required: `headingFeedback`, `forwardTranslational`, `strafeTranslational`, `brake`, `coast`, `linearBrakeCoefficients`, `quadraticBrakeCoefficients`, `headingBrakeCoefficients`, `maxAchievableForwardVelocity`, `maxAchievableStrafeVelocity`, `naturalForwardDeceleration`, `naturalStrafeDeceleration`.

If Constants already sets every required field for the classes actually involved and it still crashes, the unset field is elsewhere in the call chain (e.g. a tuner building `Localizer`+`Drivetrain` directly never touches `foresightConfig`).

`Follower.manual(double,double,double)` is a full overwrite, not a merge — compute forward/strafe/turn as locals and call exactly once per tick. `ManualDrive.fieldCentric(...)` only rotates the vector, safe to compose.

## Gradle / follower-stub diagnosis

Branch forked from old history missing gradle fixes (symptoms: AGP sync errors, RC app crash-loop with Control Hub blinking blue): `git diff <branch> master -- '*.gradle' '.idea/*'`. Expected diff is exactly: `FtcRobotController/build.gradle` `compileSdkVersion 30` → `compileSdk 34`, and `build.dependencies.gradle` removing `implementation 'com.pedropathing:ftc:2.1.2'`. Before removing: `grep -r 'com\.pedropathing\.ftc' .` must be clean — the orphaned dep transitively bumps `core:2.1.2` → `3.0.0`, bundling binary-incompatible bytecode into the same dex. Biobuzz's `pedro/` package (3.0.0 revhub API: `Follower.follow(Path)/.pose()/.manual()`, `com.pedropathing.api.Paths`) must NOT be reverted to classic `ftc:2.1.2`.

## Missing Ivy / Constants

`com.pedropathing:ftc` does NOT bundle Ivy. `com.pedropathing.ivy.*` unresolved (`Scheduler`, `Command`, `PedroCommands`, `Groups`, `sequential`/`follow`): either add `implementation 'com.pedropathing:ivy:1.0.0'` (verified on Maven Central), or refactor to the docs page's no-Ivy FSM alternative (`Timer` + `pathState` switch calling `follower.followPath(chain[, holdEnd])` once per case, advancing on `!follower.isBusy()`).

`Cannot resolve symbol 'Constants'`: it's project-authored, not a library class — create it in the same package as the OpMode (see docs "Example Constants" page). Doc values are untuned placeholders; flag with TODO for real tuning.

## @Tuner web UI is not a TeleOp

`com.pedropathing:tuning:1.0.0` `@Tuner` procedures never appear in the DS OpMode list. `TunerScanner` registers them at RC boot; `Hooks.java` starts a NanoHTTPD `WebServer` on **port 10158**. Reach it at `http://<hub-ip>:10158` (typical Control Hub AP: `192.168.43.1`) from a device on robot WiFi. Unreachable URL = RC app crashed/never booted (check `adb logcat`; suspect the stale `ftc:2.1.2` dep) — not a tuner setup problem. Confirm `implementation 'com.pedropathing:tuning:1.0.0'` alongside `com.pedropathing:revhub:3.0.0`.
