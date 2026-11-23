---
name: ftc-loop-time
description: "Biobuzz FTC OpMode loop or I/O cycle too slow (BozoAuto/BozoTeleOp): telemetry cost, I/O write budget, expensive reads; not general FTC."
---

Context: since 2026-11 all hardware traffic runs on `io/IoThread` (skill ftc-io-thread); the loop blocks in `io.awaitNewCycle()`, so loop period = I/O cycle. Hubs are `BulkCachingMode.MANUAL`, cleared once per I/O cycle. Telemetry shows "loop time" (compute only, after the await) and "I/O cycle (ms)"; `io.status()` (debug) lists per-device write/read totals.

## Slow I/O cycle

- Find the costly device in `io.status()` "total read"/"total write". Non-bulk reads (motor current, hub voltage, I2C color, Pinpoint) are >= 2 ms each: lengthen their `ReadPeriod`.
- Lower `Tunables.ioWriteBudgetMs` to shorten cycles (low-score writes wait); `WritePolicy.IMMEDIATE` devices still write every change while `softScheduling` is on.

## Slow loop compute (historical recipe, BozoAuto 2026-10)

1. **Throttle telemetry** — likeliest culprit (Panels + driver station). Gate `telemetryM.update(telemetry)` behind a `Timer` and `Tunables.telemetryIntervalMillis` (100). `Tunables.isDebugging` defaults true.
2. **Flywheel**: readings come from the I/O thread's per-cycle encoder snapshot; no caching needed in `Flywheel`.
3. **Pose**: call `follower.pose()` once per telemetry push; don't write `HandoffState.pose` every loop, only in `stop()`.
4. **Loop-time metric**: read the timer *before* reset so it includes telemetry cost; reset in `start()` so init time isn't reported.
5. Pinpoint I2C is on the I/O thread now; `follower.update()` only latches the newest sample.

## Verify

`./gradlew :TeamCode:compileDebugJavaWithJavac --offline -q` (~3s). On-robot timing must be checked by the user.
