---
name: gradle-agp-mismatch
description: "Android Studio Gradle sync fails with AGP-vs-Gradle internal-API error naming a version that doesn't match gradle-wrapper.properties; CLI ./gradlew works."
---

## Symptom

`./gradlew` works, but IDE sync fails, e.g.:

```
Failed to apply plugin 'com.android.internal.library'.
> Plugin 'com.android.internal.library' relies on 'org.gradle.api.problems.internal.InternalProblems',
  a Gradle internal API that was removed in Gradle 9.6.0.
```

The version in the error doesn't match `gradle/wrapper/gradle-wrapper.properties`. AGP 8.13.2 is incompatible with Gradle >= 9.6.0 — only with the pinned wrapper version.

## Root cause

`.idea/gradle.xml` controls the IDE's Gradle distribution — it does NOT always follow the wrapper: either no `distributionType` (IDE defaults to bundled Gradle) or `distributionType=LOCAL` with a stale `myGradleHome` under `~/.gradle/wrapper/dists/gradle-X.Y.Z-bin/...`. Confirm by comparing `distributionUrl` against the error's version and listing `~/.gradle/wrapper/dists/`.

## Fix

Set `distributionType` to `DEFAULT_WRAPPED`, remove any `myGradleHome`:

```xml
<GradleProjectSettings>
  <option name="testRunner" value="CHOOSE_PER_TEST" />
  <option name="distributionType" value="DEFAULT_WRAPPED" />
  <option name="externalProjectPath" value="$PROJECT_DIR$" />
  <option name="gradleJvm" value="#GRADLE_LOCAL_JAVA_HOME" />
  <option name="modules">
    <set>
      <option value="$PROJECT_DIR$" />
      <option value="$PROJECT_DIR$/FtcRobotController" />
      <option value="$PROJECT_DIR$/TeamCode" />
    </set>
  </option>
</GradleProjectSettings>
```

## Verify

Don't trust IDE re-sync alone: `./gradlew -version`, then `./gradlew :FtcRobotController:help` (BUILD SUCCESSFUL), then File → Sync Project with Gradle Files.
