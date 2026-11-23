# RobotPosition User Guide

## Overview

`RobotPosition` is the pose management class in the FTC robot codebase, built on the Road Runner framework for live robot pose tracking. It uses the singleton pattern and provides a unified interface for robot position and velocity queries.

## Initialization

Before use, you must call the init method:

```java
// Option 1: specify the initial pose (x=0, y=0, heading=0)
RobotPosition RobotPositioninit(HardwareMap hardwareMap, Pose2d initpose);

// Option 2: no initial pose (defaults to the origin)
RobotPosition RobotPositioninit(HardwareMap hardwareMap, null);
```

**Example:**

```java
// In the OpMode init phase
RobotPosition.getInstance().RobotPositioninit(hardwareMap, new Pose2d(0, 0, 0));
```

## Getting the Instance

```java
RobotPosition position = RobotPosition.getInstance();
```

> **Note:** Before calling `getInstance()`, you must have called `RobotPositioninit()` for initialization, otherwise an `IllegalStateException` is thrown.

## Core Methods

### Position Update (call every frame)

```java
Pose2d update()
```

Call this method in the `loop()` cycle to update the robot's current position:

```java
@Override
public void loop() {
    RobotPosition.getInstance().update();
    // Fetch position info below
}
```

### Position Queries

| Method | Return | Description |
|------|--------|------|
| `getPose2d()` | `Pose2d` | Get the full pose object |
| `getX()` | `double` | Get the X coordinate (inches) |
| `getY()` | `double` | Get the Y coordinate (inches) |
| `getTheta()` | `double` | Get the heading angle (radians) |

### Goal Selection (for auto-aim / front-aim use)

| Method | Description |
|------|------|
| `getGoalPose()` | Currently targeted goal point (only x/y valid; heading meaningless), refreshed every frame by `update()` |
| `isGoalDown()` | Whether currently aiming at the AudienceDown-side goal |
| `setManualGoalDown(boolean)` | Operator manually selects the goal (mapped to GP2.LT/RT in TeleOp) |

Selection priority (refreshed on every `update()`):

1. **This frame's non-MIDDLE MT1 observation** (`AUDIENCE_UP` / `AUDIENCE_DOWN`) -> decides the goal directly (lifted side = shoot that side);
2. Otherwise keep the previous frame's choice: the most recent non-MIDDLE reading, or the operator setting via `setManualGoalDown()`;
3. Never observed a lifted side and never manually set -> default to AudienceUp.

> Note: the manual setting should be called **before** `update()` in the same frame. `update()` only overwrites the manual value when MT1 solves a non-MIDDLE observation this frame; the manual value is kept when MT1 has no reading or reads MIDDLE.
>
> Front-aim with GP1.LT/RT in TeleOp does **not** go through here (it never calls `setManualGoalDown`); it only picks the front goal temporarily and does not affect `getGoalPose()`.

### Velocity Queries

| Method | Return | Description |
|------|--------|------|
| `getVx()` | `double` | X-axis linear velocity (inches/second) |
| `getVy()` | `double` | Y-axis linear velocity (inches/second) |
| `getOmega()` | `double` | Angular velocity (radians/second) |

### IMU Interface (heading)

The IMU reuses the already-initialized Road Runner instance (device name `"imu"`, orientation configured in `MecanumDrive.PARAMS`).

| Method | Return | Description |
|------|--------|------|
| `getYawPitchRollAngles()` | `YawPitchRollAngles` | Get the IMU yaw/pitch/roll angles |
| `getYaw(AngleUnit)` | `double` | Get yaw (heading) in the given unit |
| `resetYaw()` | `void` | Reset the IMU yaw to 0 |

> **Note:** `resetYaw()` zeroes the IMU yaw. Because Road Runner localization integrates IMU yaw for heading increments, calling it while running corrupts the pose estimate; use it only at init / calibration time.

## Usage Examples

### Basic Localization Query

```java
// In the loop() of TeleOp or Autonomous
public void loop() {
    RobotPosition.getInstance().update();

    double x = RobotPosition.getInstance().getX();
    double y = RobotPosition.getInstance().getY();
    double theta = RobotPosition.getInstance().getTheta();

    telemetry.addData("X", x);
    telemetry.addData("Y", y);
    telemetry.addData("Heading", theta);
    telemetry.update();
}
```

### Using with Road Runner Actions

```java
// Get the current pose for path planning
Pose2d currentPose = RobotPosition.getInstance().getPose2d();

// Follow the trajectory
RoadRunnerTrajectory trajectory = drive.trajectoryBuilder(currentPose)
    .splineTo(new Vector2d(24, 24), Math.PI / 2)
    .build();
drive.followTrajectory(trajectory);
```

## Dependencies

- **Road Runner**: uses `MecanumDrive` and `Localizer` for pose estimation
- **Hardware**: relies on `HardwareMap` for sensor and motor configuration

## Notes

1. **Init timing**: finish initialization in the OpMode `init()` phase
2. **Update rate**: `update()` must be called every frame in the main loop to keep data fresh
3. **Frame**: Road Runner uses inches by default, X forward, Y to the left
4. **Heading**: radians, counterclockwise positive
