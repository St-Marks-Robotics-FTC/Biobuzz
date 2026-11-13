---
name: ftc-subsys-conventions
description: "Adding new FTC subsystem classes under teamcode/subsys (motors/servos); style, PIDF reuse, live-editable Tunables.java."
---

Applies to `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/subsys/` and any magic numbers governing robot *behavior* (gains, deadbands, slew rates, thresholds, positions, ranges) anywhere in `teamcode/`.

## Core rule: behavior-tuning constants live in Tunables.java

`TeamCode/.../teamcode/Tunables.java` is `@Configurable` (byLazar Panels) and the single source of truth for anything a mentor/driver might adjust without recompiling: PIDF gains, deadbands, slew rates, multipliers, servo positions, shoot-range/angle thresholds (`field/GoalTargeting.java` reads these via `import static ...Tunables.*`), drive-shaping constants even in "standalone" OpModes like `WilsTeleOp`. Fields MUST be `public static` non-final — Panels can't edit `final` or instance fields.

Does NOT belong in Tunables: physical hardware facts (e.g. `Flywheel.TICKS_PER_REV` encoder resolution — live-editing produces wrong math); fixed game-manual field geometry (`FieldConstants.Goal` positions/facings); vendored/adapted PedroPathing AutoTune procedure internals under `pedro/procedures/` (e.g. `ForesightTuner`'s `POWER`/`RUNTIME`/`SAMPLES`).

## Live-reload pattern

Reading a Tunables field once in a constructor only picks up the startup value. Subsystems must re-read tunable fields every tick — e.g. `Flywheel.update()` re-reads `Tunables.flywheel{P,I,D,F}` each call — and the *caller* must invoke `update()` every `loop()`, not just on state changes.

## Subsystem shape

```java
import static org.firstinspires.ftc.teamcode.Tunables.*;

public class Foo {
    public enum State {OFF, FORWARD, REVERSE} // or OPEN/CLOSED for servos

    private final DcMotor motor; // or Servo
    private State state = State.OFF;

    public Foo(HardwareMap hw) {
        motor = hw.get(DcMotor.class, "lowercasename"); // hardware map name matches README's config
    }

    public State getState() { return state; }

    public void forward() { state = State.FORWARD; motor.setPower(1.0); } // setter mutates state AND commands hardware
}
```

Minimal one-line comments, no large blocks. Constructor takes `HardwareMap hw`, looks up by exact lowercase name. Setters are verbs (`forward()/reverse()/off()`, `open()/close()`), never JavaBean-style, always state+hardware together.

## Verify

No automated tests in this repo. `./gradlew :TeamCode:compileDebugJavaWithJavac -q` after any Tunables/subsys change.
