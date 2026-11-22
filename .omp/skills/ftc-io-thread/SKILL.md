---
name: ftc-io-thread
description: "Adding/changing hardware access in Biobuzz TeamCode (motors, servos, sensors, Pinpoint, new device types like distance sensors) through the io/ IoThread; loop/I/O threading rules."
---

All hub, I2C and Pinpoint traffic runs on one background thread: `teamcode/io/IoThread.java`. OpModes and subsystems never touch `DcMotorEx`/`Servo`/sensors directly; they talk to `IoDevice` subclasses in `teamcode/io/`.

## OpMode lifecycle (every OpMode, tuners included)

```java
io = new IoThread(hardwareMap);      // sets every LynxModule to BulkCachingMode.MANUAL
robot = new Robot(io);               // subsystems build devices; devices self-register
follower = Constants.create(io);     // PinpointDevice (Localizer) + MecanumDevice (Drivetrain)
follower.setPose(...);
io.start();                          // one synchronous cycle (all reads, all writes: init servo positions move), then the thread
// loop(): io.awaitNewCycle() FIRST, then logic. Loop runs at the I/O cycle rate on fresh readings.
// stop(): commands, then io.stop()  -> joins the thread, then flushes pending commands on the caller thread.
// LinearOpMode: try { ... } finally { io.stop(); }
```

Devices must be constructed before `io.start()` (register throws after). An `OpModeManagerNotifier` post-stop listener halts the thread (no flush) if `stop()` was skipped or the OpMode crashed. I/O thread exceptions are rethrown from `awaitNewCycle()`; no cycle for `Tunables.ioStallTimeoutMs` also throws.

## Cycle order and scheduling

reads (clear bulk caches; each device's `readIfDue`) -> publish + wake loop -> writes by score within `Tunables.ioWriteBudgetMs`. Score = `priority * change * (1 + waitingMs / stalenessReduction)`; `softScheduling` forces devices past `WritePolicy` max staleness (`WritePolicy.IMMEDIATE` = 0 ms: drivetrain, Pinpoint pose) through regardless of budget.

## Why one I/O thread (researched 2026-11, SDK 12.0.0 bytecode + gm0)

`LynxRespondable.send()/sendReceive()` hold `LynxUsbDeviceImpl.networkTransmissionLock` for the whole request/response; there is one lock per USB/UART link. An Expansion Hub chained over RS485 shares the Control Hub's link, so per-hub threads would just wait on the lock. Only a USB-connected Expansion Hub has its own link (and USB commands are slower: ~3 ms vs ~2 ms UART). Limelight3A already polls on its own executor; `getLatestResult()` is a volatile read, so `Vision` stays on the loop thread.

## Adding a new device type (e.g. Pololu distance sensor)

Subclass `IoDevice` **in the `io` package** (hooks are package-private on purpose):
- Constructor (loop thread, pre-start): `super(io, name)` (read-only) or `super(io, name, writePolicy, initialWriteEstimateMs)`; `io.hw().get(...)`; one-time hardware config.
- `readIfDue(now)`: check each value's `ReadPeriod.isDue(hasRead, lastReadNanos, now)`, read, publish an immutable reading (`TimedValue` or a small final class with `nanoTime`) to a `volatile` field.
- `pendingChange()` in [0,1], must be 1 until the first write; `write()` reads the `volatile` command once and sends it.
- Public getters throw `IllegalStateException` when a reading doesn't exist (read disabled / not started) — never return defaults.
Every non-bulk read costs >= one Lynx command (~2 ms): choose slow `ReadPeriod`s (encoder position/velocity is free via the bulk read).

Pedro integration: `PinpointDevice` wraps Pedro's `PinpointLocalizer`, `MecanumDevice` wraps Pedro's `Mecanum` (owned by the I/O thread). `MecanumDevice.maxScaling/interpolateVelocity` delegate to Mecanum because they are pure in revhub 3.0.2 — re-check with `javap -c` if Pedro is upgraded. TeleOp drives via `follower.manual(...)` (no separate Drive subsystem).

## Verify

`./gradlew :TeamCode:compileDebugJavaWithJavac --offline -q`; scheduler/ReadPeriod unit tests: `JAVA_HOME=/usr/lib/jvm/java-21-openjdk ./gradlew :TeamCode:testDebugUnitTest --offline` (system JDK 27 breaks Gradle 9.1 build-script compilation). On-robot timing (I/O cycle ms telemetry) must be checked by the user.
