package org.firstinspires.ftc.teamcode.tuner;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

@TeleOp(name="VisionTuner", group="Tuner")
public class VisionTuner extends LinearOpMode {
    private enum TuneMode {
        BLUE,
        RED
    }

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap, true);
        TelemetryManager telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        Follower follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(72, 72, Math.toRadians(90))); // start in center of field

        TuneMode mode = TuneMode.BLUE;
        boolean isCapturing = false;
        double lastLoggedFrame = Double.NaN;
        int rowsLogged = 0;

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            if (gamepad1.rightBumperWasPressed()) {
                mode = TuneMode.BLUE;
                vision.startPipeline(Vision.Pipeline.BLUE);
            }
            if (gamepad1.leftBumperWasPressed()) {
                mode = TuneMode.RED;
                vision.startPipeline(Vision.Pipeline.RED);
            }

            if (gamepad1.startWasPressed()) {
                isCapturing = !isCapturing;
                if (isCapturing) {
                    rowsLogged = 0;
                    lastLoggedFrame = Double.NaN;
                    RobotLog.d("VisionTuner: CAPTURE START " + mode);
                    RobotLog.d("VisionTuner: odoX, odoY, rawX, rawY, rawZ, fixX, fixY, fixZ, rawErrX, rawErrY, rawErrZ, fixErrX, fixErrY, fixErrZ");
                } else {
                    RobotLog.d("VisionTuner: CAPTURE STOP " + rowsLogged + " rows");
                }
            }

            vision.update();

            Pose odo = follower.pose();
            Position raw = vision.getLastRawPosition();
            Vision.VisionResult fixed = vision.getResult();

            if (isCapturing && raw != null && vision.getLastRawFrameStamp() != lastLoggedFrame) {
                lastLoggedFrame = vision.getLastRawFrameStamp();

                double fixX = Double.NaN, fixY = Double.NaN, fixZ = Double.NaN;
                if (fixed != null) {
                    fixX = fixed.pose().x();
                    fixY = fixed.pose().y();
                    fixZ = 0; // the hive fix puts the robot back on the floor
                }

                RobotLog.d("VisionTuner: " + odo.x() + ", " + odo.y() + ", "
                        + raw.x + ", " + raw.y + ", " + raw.z + ", "
                        + fixX + ", " + fixY + ", " + fixZ + ", "
                        + (raw.x - odo.x()) + ", " + (raw.y - odo.y()) + ", " + (raw.z - 0) + ", "
                        + (fixX - odo.x()) + ", " + (fixY - odo.y()) + ", " + (fixZ - 0));
                rowsLogged++;
            }

            telemetryM.addLine("right bumper: BLUE, left bumper: RED");
            telemetryM.addLine("start: toggle capture");
            telemetryM.addData("side", mode);
            telemetryM.addData("capturing", isCapturing);
            telemetryM.addData("rows logged", rowsLogged);
            telemetryM.addData("odo pos", odo.toString());
            if (raw != null) telemetryM.addData("raw pos", raw.toString());
            telemetryM.addData("fixed pos", fixed == null ? "rejected / no frame" : fixed.pose().toString());
            telemetryM.addData("last hive state", vision.getLastHiveState());

            telemetryM.update(telemetry);

            idle();
        }
    }
}
