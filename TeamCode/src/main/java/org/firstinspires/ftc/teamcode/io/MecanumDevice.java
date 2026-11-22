package org.firstinspires.ftc.teamcode.io;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;

import java.util.HashMap;
import java.util.Map;

/**
 * Pedro drivetrain whose four motor writes run on the I/O thread.
 *
 * <p>The I/O thread drives Pedro's own {@link Mecanum} (wheel mixing, normalization, brake modes and its
 * {@code powerThreshold} write deadband all come from the same {@link MecanumConfig}). The loop thread's
 * {@link #drive} and {@link #stop} only record the newest command.
 */
public final class MecanumDevice extends IoDevice implements Drivetrain {
    public static final String NAME = "drivetrain";
    private static final double INITIAL_WRITE_ESTIMATE_MILLIS = 8; // four motor powers, one Lynx command each

    /** newest thing the follower asked the wheels to do. Immutable. */
    private static final class Command {
        final boolean isStop;
        final DrivePowers powers; // zero for stop
        final boolean brake;

        Command(boolean isStop, DrivePowers powers, boolean brake) {
            this.isStop = isStop;
            this.powers = powers;
            this.brake = brake;
        }
    }

    // pure math (maxScaling, interpolateVelocity) is safe from the loop thread; everything else is I/O thread only
    private final Mecanum mecanum;
    private final boolean manualBrakeMode;

    // loop thread -> I/O thread
    private volatile Command command;

    // I/O thread only
    private Command ioWritten;

    /** builds Pedro's Mecanum, which sets motor directions and FLOAT; that state counts as written */
    public MecanumDevice(IoThread io, MecanumConfig config) {
        super(io, NAME, WritePolicy.IMMEDIATE, INITIAL_WRITE_ESTIMATE_MILLIS);
        mecanum = new Mecanum(io.hw(), config);
        manualBrakeMode = config.manualBrakeMode.get();
        command = new Command(true, DrivePowers.zero(), false); // what the Mecanum constructor left the motors at
        ioWritten = command;
    }

    @Override
    public void drive(DrivePowers powers, boolean useBrakeMode) {
        if (powers == null) throw new IllegalArgumentException("drive powers are null");
        command = new Command(false, powers, useBrakeMode);
    }

    @Override
    public void stop() {
        stop(manualBrakeMode);
    }

    @Override
    public void stop(boolean useBrakeMode) {
        command = new Command(true, DrivePowers.zero(), useBrakeMode);
    }

    /** pure wheel-power math on Pedro's Mecanum (verified against revhub 3.0.2: reads only its arguments) */
    @Override
    public double maxScaling(DrivePowers current, DrivePowers next) {
        return mecanum.maxScaling(current, next);
    }

    /** pure math on Pedro's Mecanum (verified against revhub 3.0.2: reads only its arguments) */
    @Override
    public double interpolateVelocity(double forward, double strafe, double angle) {
        return mecanum.interpolateVelocity(forward, strafe, angle);
    }

    /** the newest command, not what the motors received */
    @Override
    public Map<String, Object> debug() {
        Command newest = command;
        Map<String, Object> out = new HashMap<>();
        out.put("stop", newest.isStop);
        out.put("brake", newest.brake);
        out.put("forward", newest.powers.forward());
        out.put("strafe", newest.powers.strafe());
        out.put("turn", newest.powers.turn());
        return out;
    }

    @Override
    double pendingChange() {
        Command newest = command;
        Command written = ioWritten;
        if (newest == written) return 0;
        if (newest.isStop != written.isStop || newest.brake != written.brake) return 1;
        double change = Math.max(Math.abs(newest.powers.forward() - written.powers.forward()),
                Math.max(Math.abs(newest.powers.strafe() - written.powers.strafe()),
                        Math.abs(newest.powers.turn() - written.powers.turn())));
        return Math.min(1, change / 2); // each axis spans [-1, 1]; halve so a full reversal scores 1
    }

    @Override
    void write() {
        Command newest = command;
        if (newest.isStop) mecanum.stop(newest.brake);
        else mecanum.drive(newest.powers, newest.brake);
        ioWritten = newest;
    }
}
