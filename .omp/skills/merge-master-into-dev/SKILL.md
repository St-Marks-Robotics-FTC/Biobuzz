---
name: merge-master-into-dev
description: "Merging master into a Biobuzz dev branch (dev-wbrous/dev-bang) with conflicts; BozoAuto compile checks after merge."
---

New branches get cut from commits predating the gradle fixes on `master` — always `git diff <branch> master -- '*.gradle' '.idea/*'` early (expected: `compileSdkVersion 30` → `compileSdk 34`, removal of `com.pedropathing:ftc:2.1.2`).

After merging, deps may be bumped (e.g. `com.pedropathing:revhub`) and `--offline` fails to resolve — rerun without `--offline`. Post-merge "cannot resolve method X in Robot" usually means the other branch renamed it (e.g. `Robot.updateLaunch()` -> `Robot.update()`); grep `Robot.java` and the TeleOp caller for the new name.

Verify: `cd <repo> && JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew -q :TeamCode:compileDebugJavaWithJavac` (real exit code, not a grep pipe's; force JDK 21 — system default 27 breaks with "Unsupported class file major version 71"). No hardware here: on-robot behavior untested. Leave Blue/Red tuning edits uncommitted unless asked.
