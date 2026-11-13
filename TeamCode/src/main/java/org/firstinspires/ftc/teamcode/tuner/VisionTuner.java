package org.firstinspires.ftc.teamcode.tuner;

import static com.pedropathing.api.Paths.*;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.FieldGoals;
import org.firstinspires.ftc.teamcode.Tunables;
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
        follower.setPose(new Pose(Tunables.visionTunerSpotX, Tunables.visionTunerSpotY, Math.toRadians(90)));
        VisionDataCollector collector = new VisionDataCollector(follower, vision);

        TuneMode mode = TuneMode.BLUE;
        boolean isCalibrating = false;
        int calibrationSpot = 1;

        TuneMode mode0 = TuneMode.values()[0];
        TuneMode mode1 = TuneMode.values()[1];

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            telemetryM.addLine("select your tuning mode using the gamepad:");
            telemetryM.addLine("right bumper: " + mode0);
            telemetryM.addLine("left bumper: " + mode1);

            if (gamepad1.rightBumperWasPressed()) {
                mode = mode0;
                vision.startPipeline(Vision.Pipeline.BLUE);
            }
            if (gamepad1.leftBumperWasPressed()) {
                mode = mode1;
                vision.startPipeline(Vision.Pipeline.RED);
            }


            if (gamepad1.startWasPressed()) {
                if (collector.isRunning()) collector.stop();
                else collector.start();
            }
            if (gamepad1.backWasPressed()) isCalibrating = !isCalibrating;
            if (gamepad1.dpadUpWasPressed()) calibrationSpot++;
            if (gamepad1.dpadDownWasPressed() && calibrationSpot > 1) calibrationSpot--;
            if (gamepad1.aWasPressed() && !collector.isRunning()) {
                follower.setPose(new Pose(Tunables.visionTunerSpotX, Tunables.visionTunerSpotY, follower.pose().heading()));
            }

            telemetryM.addLine("");
            telemetryM.addLine("current testing mode is: " + mode);
            telemetryM.addLine("START: start/stop automatic data collection");
            telemetryM.addLine("toggle calibration with BACK button");
            telemetryM.addLine("collector: " + collector.status());
            telemetryM.addLine("calibration is: " + isCalibrating);
            telemetryM.addLine("calibration spot (dpad up/down): S" + calibrationSpot);
            telemetryM.addLine("A: set pose to visionTunerSpotX/Y");

            vision.update(follower.pose().heading());
            collector.update();

            Position rawPos = vision.getLatestResult().getBotpose().getPosition();

            telemetryM.addData("raw position", rawPos.toString());
            telemetryM.addData("last botpose", vision.getLastBotPose().toString());
            telemetryM.addData("last hive state", vision.getLastHiveState());
            telemetryM.addData("staleness", vision.getStaleness());
            telemetryM.addData("odo pos", follower.pose().toString());
            telemetryM.addData("pipeline", vision.getPipeline());

            telemetryM.addData("diff y", follower.pose().y() - vision.getLastBotPose().y());
            telemetryM.addData("Tags seen", vision.getLatestResult().getBotposeTagCount());

            if (isCalibrating) {
                LLResult result = vision.getLatestResult();
                if (vision.isReasonable(result)) {
                    Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
                    double y = 72 - botpose.x; // convert to pedro units
                    double z = botpose.z;
                    double avgDist = result.getBotposeAvgDist() * 39.37; // m -> in

                    RobotLog.d("VisionTuner: " + mode + calibrationSpot + ", " + follower.pose().y() + ", " + y + ", " + z + ", " + avgDist);
                }
            }

            telemetryM.update(telemetry);

            idle();
        }

        collector.stop();
    }

    static public class VisionDataCollector {
        private static final double FIELD_SIZE = 144;
        private static final double SEGMENT_STEP = 0.5;
        private static final double EPSILON = 1e-6;
        private static final double MIN_LEG_LENGTH = 0.5;
        private static final double STALL_SPEED = 1.5; // in/s
        private static final int POSE_BUFFER_SIZE = 256;
        private static final long FLUSH_INTERVAL_NANOS = 1_000_000_000L;

        // TRAVEL to spot, SPIN there, repeat without stopping
        private enum State {
            IDLE,
            TRAVEL,
            SPIN,
            DONE
        }

        private final Follower follower;
        private final Vision vision;
        private final Timer legTimer = new Timer();
        private final Timer spinTimer = new Timer();

        private State state = State.IDLE;
        private String status = "idle";

        private double clearance;
        private double[] xs, ys;
        private boolean[][] navigable; // robot may drive here
        private boolean[][] isSpot; // navigable and outside no-record band
        private List<int[]> targets;
        private int targetIdx;
        private int[] curNode;

        // current spot and heading sweep (radians, unwrapped)
        private double spinX, spinY, spinFrom, spinTo;

        // odometry history for matching camera frames
        private final long[] poseNanos = new long[POSE_BUFFER_SIZE];
        private final double[] poseX = new double[POSE_BUFFER_SIZE];
        private final double[] poseY = new double[POSE_BUFFER_SIZE];
        private final double[] poseHeading = new double[POSE_BUFFER_SIZE];
        private final double[] poseOut = new double[3];
        private int poseNext, poseCount;

        private double lastFrameTime = Double.NaN;
        private long lastFlushNanos;

        private int rows, dropped, inBand, noPose, skippedSpots, validFrames, lastTags;
        private BufferedWriter writer;
        private String filePath = "";

        public VisionDataCollector(Follower follower, Vision vision) {
            this.follower = follower;
            this.vision = vision;
        }

        public boolean isRunning() {
            return state == State.TRAVEL || state == State.SPIN;
        }

        public String status() {
            if (!isRunning() && state != State.DONE) return status;
            return status + (isRunning() && inNoRecordBand(follower.pose().y()) ? " [no-record band]" : "")
                    + " | spot " + Math.min(targetIdx + 1, targets.size()) + "/" + targets.size()
                    + ", rows " + rows + ", dropped " + dropped + ", inBand " + inBand + ", noPose " + noPose
                    + ", validFrames " + validFrames + ", lastTags " + lastTags
                    + ", skippedSpots " + skippedSpots
                    + ", file " + filePath;
        }

        // safety checks, grid, csv, then first leg
        public void start() {
            clearance = Math.hypot(Tunables.visionTunerRobotLength, Tunables.visionTunerRobotWidth) / 2
                    + Tunables.visionTunerWallMargin;
            buildGrid();

            Pose pose = follower.pose();
            if (!isPointSafe(pose.x(), pose.y())) {
                refuse();
                return;
            }
            int[] nearest = nearestNode(pose.x(), pose.y());
            if (nearest == null || !isSegmentSafe(pose.x(), pose.y(), xs[nearest[0]], ys[nearest[1]])) {
                refuse();
                return;
            }

            targets = serpentine();
            if (targets.isEmpty()) {
                status = "START REFUSED: no valid spots";
                return;
            }

            rows = 0;
            dropped = 0;
            inBand = 0;
            noPose = 0;
            skippedSpots = 0;
            targetIdx = 0;
            lastFrameTime = Double.NaN;
            validFrames = 0;
            lastTags = 0;
            poseNext = 0;
            poseCount = 0;
            lastFlushNanos = System.nanoTime();

            try {
                File dir = new File(AppUtil.FIRST_FOLDER, "vision_data");
                if (!dir.exists() && !dir.mkdirs()) throw new IOException("mkdirs failed");
                File file = new File(dir, "vision_data_"
                        + new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date()) + ".csv");
                filePath = file.getAbsolutePath();
                writer = new BufferedWriter(new FileWriter(file));
                writer.write("pedroX,pedroY,pedroHeading,pedroZ,limelightX,limelightY,limelightZ,limelightHeading,tagCount");
                writer.newLine();
                writer.flush();
            } catch (IOException e) {
                status = "ABORTED: CSV open failed";
                writer = null;
                return;
            }

            curNode = nearest;
            status = "running";
            advanceTarget();
        }

        // stop motors, close file
        public void stop() {
            boolean wasRunning = isRunning();
            follower.stop();
            follower.drivetrain.stop();
            closeWriter();
            if (wasRunning) {
                state = State.DONE;
                status = "stopped";
            }
        }

        // every loop: pose history, step, log frame
        public void update() {
            if (!isRunning()) return;
            recordPose();
            switch (state) {
                case TRAVEL:
                    updateTravel();
                    break;
                case SPIN:
                    updateSpin();
                    break;
                default:
                    break;
            }
            if (isRunning()) logFrame();
        }

        private void refuse() {
            status = "START REFUSED: pose not in a safe spot, press A to reposition";
        }

        private void abort(String reason) {
            follower.stop();
            follower.drivetrain.stop();
            closeWriter();
            state = State.DONE;
            status = "ABORTED: " + reason;
        }

        private void finish() {
            follower.stop();
            follower.drivetrain.stop();
            closeWriter();
            state = State.DONE;
            status = "DONE";
        }

        private void closeWriter() {
            if (writer == null) return;
            try {
                writer.close();
            } catch (IOException ignored) {
            }
            writer = null;
        }

        // full-width band: no rows, no spots
        private static boolean inNoRecordBand(double y) {
            return y >= Tunables.visionTunerNoRecordMinY - EPSILON && y <= Tunables.visionTunerNoRecordMaxY + EPSILON;
        }

        // store pose with timestamp
        private void recordPose() {
            Pose pose = follower.pose();
            poseNanos[poseNext] = System.nanoTime();
            poseX[poseNext] = pose.x();
            poseY[poseNext] = pose.y();
            poseHeading[poseNext] = pose.heading();
            poseNext = (poseNext + 1) % POSE_BUFFER_SIZE;
            if (poseCount < POSE_BUFFER_SIZE) poseCount++;
        }

        // interpolated pose at nanoTime; false if too old
        private boolean poseAt(long nanos, double[] out) {
            if (poseCount == 0) return false;
            int newer = Math.floorMod(poseNext - 1, POSE_BUFFER_SIZE);
            if (nanos >= poseNanos[newer]) {
                out[0] = poseX[newer];
                out[1] = poseY[newer];
                out[2] = poseHeading[newer];
                return true;
            }
            for (int ago = 1; ago < poseCount; ago++) {
                int older = Math.floorMod(poseNext - 1 - ago, POSE_BUFFER_SIZE);
                if (nanos >= poseNanos[older]) {
                    double f = (double) (nanos - poseNanos[older]) / (poseNanos[newer] - poseNanos[older]);
                    out[0] = poseX[older] + (poseX[newer] - poseX[older]) * f;
                    out[1] = poseY[older] + (poseY[newer] - poseY[older]) * f;
                    out[2] = AngleUnit.normalizeRadians(poseHeading[older]
                            + AngleUnit.normalizeRadians(poseHeading[newer] - poseHeading[older]) * f);
                    return true;
                }
                newer = older;
            }
            return false;
        }

        private static double frameTime(LLResult result) {
            double timestamp = result.getTimestamp();
            return timestamp != 0 ? timestamp : result.getControlHubTimeStamp();
        }

        // log new frames, skipping the band
        private void logFrame() {
            LLResult result = vision.getLatestResult();
            if (!vision.isReasonable(result) || frameTime(result) == lastFrameTime) return;
            lastFrameTime = frameTime(result);
            validFrames++;
            lastTags = result.getBotposeTagCount();
            if (lastTags < Tunables.visionTunerMinTags) {
                dropped++;
                return;
            }

            // pose at capture time, not now
            double ageMillis = Math.max(0, result.getStaleness() + result.getCaptureLatency()
                    + result.getTargetingLatency() + Tunables.visionTunerLatencyOffsetMillis);
            if (!poseAt(System.nanoTime() - (long) (ageMillis * 1e6), poseOut)) {
                noPose++;
                return;
            }
            if (inNoRecordBand(poseOut[1])) {
                inBand++;
                return;
            }

            try {
                writeRow(result, poseOut[0], poseOut[1], poseOut[2]);
                long now = System.nanoTime();
                if (now - lastFlushNanos >= FLUSH_INTERVAL_NANOS) {
                    writer.flush();
                    lastFlushNanos = now;
                }
            } catch (IOException e) {
                abort("CSV write failed");
            }
        }

        // one csv line: odometry pose + limelight pose + tags
        private void writeRow(LLResult result, double x, double y, double heading) throws IOException {
            Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
            double yaw = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
            writer.write(String.format(Locale.US, "%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%.5f,%d",
                    x, y, AngleUnit.normalizeRadians(heading), Tunables.visionTunerTrueZ,
                    botpose.y + 72, 72 - botpose.x, botpose.z, AngleUnit.normalizeRadians(yaw - Math.PI / 2),
                    result.getBotposeTagCount()));
            writer.newLine();
            rows++;
        }

        // next reachable spot, or finish
        private void advanceTarget() {
            while (targetIdx < targets.size()) {
                int[] spot = targets.get(targetIdx);
                ArrayDeque<int[]> route = bfs(curNode, spot);
                if (route != null) {
                    beginTravel(route, spot);
                    return;
                }
                skippedSpots++;
                targetIdx++;
            }
            finish();
        }

        // one continuous path; heading eases toward sweep start
        private void beginTravel(ArrayDeque<int[]> route, int[] spot) {
            Pose pose = follower.pose();
            spinX = xs[spot[0]];
            spinY = ys[spot[1]];
            planSweep(spinX, spinY, pose.heading());
            List<double[]> corners = pull(pose, route);
            curNode = spot;
            legTimer.reset();
            if (corners.size() < 2) {
                beginSpin();
                return;
            }

            // heading spread by distance along path
            double[] along = new double[corners.size()];
            for (int k = 1; k < along.length; k++) {
                along[k] = along[k - 1] + Math.hypot(corners.get(k)[0] - corners.get(k - 1)[0],
                        corners.get(k)[1] - corners.get(k - 1)[1]);
            }
            double delta = AngleUnit.normalizeRadians(spinFrom - pose.heading());
            Pose[] waypoints = new Pose[corners.size()];
            for (int k = 0; k < waypoints.length; k++) {
                waypoints[k] = new Pose(corners.get(k)[0], corners.get(k)[1],
                        pose.heading() + delta * along[k] / along[along.length - 1]);
            }
            Path[] legs = new Path[waypoints.length - 1];
            for (int k = 0; k < legs.length; k++) {
                legs[k] = line(waypoints[k], waypoints[k + 1]).linear(waypoints[k], waypoints[k + 1])
                        .with(Constants.foresightConfig.maxPathSpeed.at(Tunables.visionTunerSpeed));
            }
            follower.follow(path(legs));
            state = State.TRAVEL;
            status = "running: travel";
        }

        // fewest safe straight legs, from the real pose
        private List<double[]> pull(Pose pose, ArrayDeque<int[]> route) {
            List<double[]> points = new ArrayList<>();
            points.add(new double[]{pose.x(), pose.y()});
            points.add(new double[]{xs[curNode[0]], ys[curNode[1]]});
            for (int[] node : route) points.add(new double[]{xs[node[0]], ys[node[1]]});

            List<double[]> corners = new ArrayList<>();
            corners.add(points.get(0));
            int a = 0;
            while (a < points.size() - 1) {
                int b = points.size() - 1;
                while (b > a + 1 && !isSegmentSafe(points.get(a)[0], points.get(a)[1], points.get(b)[0], points.get(b)[1])) b--;
                double[] last = corners.get(corners.size() - 1);
                double[] next = points.get(b);
                if (Math.hypot(next[0] - last[0], next[1] - last[1]) >= MIN_LEG_LENGTH) corners.add(next);
                a = b;
            }
            return corners;
        }

        // sweep +-limit around nearest goal direction
        private void planSweep(double sx, double sy, double heading) {
            Pose[] goals = {FieldGoals.redGoalNorth, FieldGoals.redGoalSouth, FieldGoals.blueGoalNorth, FieldGoals.blueGoalSouth};
            Pose goal = goals[0];
            for (Pose g : goals) {
                if (Math.hypot(g.x() - sx, g.y() - sy) < Math.hypot(goal.x() - sx, goal.y() - sy)) goal = g;
            }
            double toGoal = Math.atan2(goal.y() - sy, goal.x() - sx);
            double limit = Math.toRadians(Math.min(Math.max(Tunables.visionTunerGoalHeadingLimitDegrees, 0), 180));
            double low = toGoal - limit;
            double high = toGoal + limit;
            // start at the end nearer the current heading
            boolean startLow = Math.abs(AngleUnit.normalizeRadians(low - heading))
                    <= Math.abs(AngleUnit.normalizeRadians(high - heading));
            spinFrom = startLow ? low : high;
            spinTo = startLow ? high : low;
        }

        // follow path; spin when near the spot
        private void updateTravel() {
            Pose pose = follower.pose();
            if (Math.hypot(pose.x() - spinX, pose.y() - spinY) <= Tunables.visionTunerSpinEnterRadius
                    || !follower.isBusy()) {
                beginSpin();
                return;
            }
            // abort only if stuck
            if (follower.velocity().linSpeed() >= STALL_SPEED) {
                legTimer.reset();
            } else if (legTimer.milliseconds() >= Tunables.visionTunerMoveTimeoutMillis) {
                abort("travel stalled");
            }
        }

        private void beginSpin() {
            state = State.SPIN;
            status = "running: spin";
            spinTimer.reset();
            follower.hold(new Pose(spinX, spinY, AngleUnit.normalizeRadians(spinFrom)));
        }

        // slide hold heading at constant rate
        private void updateSpin() {
            double sweep = spinTo - spinFrom;
            double rate = Math.toRadians(Math.max(Tunables.visionTunerSpinDegPerSec, 1));
            double fraction = Math.abs(sweep) < EPSILON ? 1 : Math.min(spinTimer.seconds() * rate / Math.abs(sweep), 1);
            follower.hold(new Pose(spinX, spinY, AngleUnit.normalizeRadians(spinFrom + sweep * fraction)));
            // sweep done: go on immediately
            if (fraction >= 1) {
                targetIdx++;
                advanceTarget();
            }
        }

        // grid: drivable nodes, and spots outside the band
        private void buildGrid() {
            xs = axis(Tunables.visionTunerSpotX);
            ys = axis(Tunables.visionTunerSpotY);
            navigable = new boolean[xs.length][ys.length];
            isSpot = new boolean[xs.length][ys.length];
            for (int i = 0; i < xs.length; i++) {
                for (int j = 0; j < ys.length; j++) {
                    navigable[i][j] = isPointSafe(xs[i], ys[j]);
                    isSpot[i][j] = navigable[i][j] && !inNoRecordBand(ys[j]);
                }
            }
        }

        // makes the grid lines for one axis, lined up with the start spot (72,72)
        private double[] axis(double anchor) {
            double min = clearance;
            double max = FIELD_SIZE - clearance;
            double spacing = Math.max(Tunables.visionTunerGridSpacing, 1);
            int kMin = (int) Math.ceil((min - anchor) / spacing - EPSILON);
            int kMax = (int) Math.floor((max - anchor) / spacing + EPSILON);
            List<Double> values = new ArrayList<>();
            addDistinct(values, min);
            for (int k = kMin; k <= kMax; k++) addDistinct(values, anchor + k * spacing);
            addDistinct(values, max);
            double[] out = new double[values.size()];
            for (int i = 0; i < out.length; i++) out[i] = values.get(i);
            return out;
        }

        private static void addDistinct(List<Double> values, double v) {
            if (values.isEmpty() || Math.abs(values.get(values.size() - 1) - v) > 1e-6) values.add(v);
        }

        // is the robot's center ok here? (inside walls + away from the obstacle boxes)
        private boolean isPointSafe(double x, double y) {
            if (x < clearance - EPSILON || x > FIELD_SIZE - clearance + EPSILON
                    || y < clearance - EPSILON || y > FIELD_SIZE - clearance + EPSILON) return false;
            return rectDistance(x, y, Tunables.visionTunerObstacleAMinX, Tunables.visionTunerObstacleAMinY,
                    Tunables.visionTunerObstacleAMaxX, Tunables.visionTunerObstacleAMaxY) >= clearance - EPSILON
                    && rectDistance(x, y, Tunables.visionTunerObstacleBMinX, Tunables.visionTunerObstacleBMinY,
                    Tunables.visionTunerObstacleBMaxX, Tunables.visionTunerObstacleBMaxY) >= clearance - EPSILON;
        }

        // walks along the line and checks each little step with isPointSafe
        private boolean isSegmentSafe(double x0, double y0, double x1, double y1) {
            double length = Math.hypot(x1 - x0, y1 - y0);
            int steps = Math.max(1, (int) Math.ceil(length / SEGMENT_STEP));
            for (int s = 0; s <= steps; s++) {
                double t = (double) s / steps;
                if (!isPointSafe(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)) return false;
            }
            return true;
        }

        // distance from a point to a rectangle (0 if inside)
        private static double rectDistance(double x, double y, double minX, double minY, double maxX, double maxY) {
            double dx = Math.max(Math.max(minX - x, 0), x - maxX);
            double dy = Math.max(Math.max(minY - y, 0), y - maxY);
            return Math.hypot(dx, dy);
        }

        // grid node closest to the given position
        private int[] nearestNode(double x, double y) {
            int[] best = null;
            double bestDist = Double.MAX_VALUE;
            for (int i = 0; i < xs.length; i++) {
                for (int j = 0; j < ys.length; j++) {
                    if (!navigable[i][j]) continue;
                    double d = Math.hypot(xs[i] - x, ys[j] - y);
                    if (d < bestDist && isSegmentSafe(x, y, xs[i], ys[j])) {
                        bestDist = d;
                        best = new int[]{i, j};
                    }
                }
            }
            return best;
        }

        // spots row by row, zigzag
        private List<int[]> serpentine() {
            List<int[]> out = new ArrayList<>();
            for (int j = 0; j < ys.length; j++) {
                boolean forward = j % 2 == 0;
                for (int k = 0; k < xs.length; k++) {
                    int i = forward ? k : xs.length - 1 - k;
                    if (isSpot[i][j]) out.add(new int[]{i, j});
                }
            }
            return out;
        }

        // can we drive from node a to node b safely?
        private boolean edgeSafe(int i0, int j0, int i1, int j1) {
            return navigable[i1][j1] && isSegmentSafe(xs[i0], ys[j0], xs[i1], ys[j1]);
        }

        // finds a path of grid nodes between two spots without crossing obstacles
        private ArrayDeque<int[]> bfs(int[] from, int[] to) {
            int nx = xs.length;
            int ny = ys.length;
            int[][] prev = new int[nx][ny];
            boolean[][] seen = new boolean[nx][ny];
            ArrayDeque<int[]> queue = new ArrayDeque<>();
            queue.add(from);
            seen[from[0]][from[1]] = true;
            prev[from[0]][from[1]] = -1;
            boolean found = from[0] == to[0] && from[1] == to[1];
            while (!queue.isEmpty() && !found) {
                int[] cur = queue.poll();
                for (int di = -1; di <= 1 && !found; di++) {
                    for (int dj = -1; dj <= 1; dj++) {
                        int ni = cur[0] + di;
                        int nj = cur[1] + dj;
                        if ((di == 0 && dj == 0) || ni < 0 || nj < 0 || ni >= nx || nj >= ny || seen[ni][nj]) continue;
                        if (!edgeSafe(cur[0], cur[1], ni, nj)) continue;
                        seen[ni][nj] = true;
                        prev[ni][nj] = cur[0] * ny + cur[1];
                        if (ni == to[0] && nj == to[1]) {
                            found = true;
                            break;
                        }
                        queue.add(new int[]{ni, nj});
                    }
                }
            }
            if (!found) return null;
            ArrayDeque<int[]> route = new ArrayDeque<>();
            int i = to[0];
            int j = to[1];
            while (!(i == from[0] && j == from[1])) {
                route.addFirst(new int[]{i, j});
                int p = prev[i][j];
                i = p / ny;
                j = p % ny;
            }
            return route;
        }
    }
}
