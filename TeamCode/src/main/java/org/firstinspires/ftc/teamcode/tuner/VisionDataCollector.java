package org.firstinspires.ftc.teamcode.tuner;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.utils.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.Tunables;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsys.Vision;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Date;
import java.util.List;
import java.util.Locale;

public class VisionDataCollector {
    private static final double FIELD_SIZE = 144;
    private static final double SEGMENT_STEP = 0.5;
    private static final double EPSILON = 1e-6;
    private static final double MIN_LEG_LENGTH = 0.5;

    private enum State {
        IDLE,
        TRAVEL,
        TURN,
        SETTLE,
        SAMPLE,
        DONE
    }

    private final Follower follower;
    private final Vision vision;
    private final Timer stateTimer = new Timer();
    private final Timer legTimer = new Timer();

    private State state = State.IDLE;
    private String status = "idle";

    private double clearance;
    private double[] xs, ys;
    private boolean[][] valid;
    private List<int[]> targets;
    private int targetIdx;
    private int[] curNode;
    private ArrayDeque<int[]> path;
    private boolean legActive;
    private int[] legNode;

    private int headingCount;
    private int headingIdx;
    private boolean holdStarted;

    private int samplesThisHeading;
    private double lastFrameTime = Double.NaN;

    private int rows, dropped, skippedSpots, skippedHeadings;
    private BufferedWriter writer;
    private String filePath = "";

    public VisionDataCollector(Follower follower, Vision vision) {
        this.follower = follower;
        this.vision = vision;
    }

    public boolean isRunning() {
        return state != State.IDLE && state != State.DONE;
    }

    public String status() {
        if (targets == null || (!isRunning() && state != State.DONE)) return status;
        return status + " | spot " + Math.min(targetIdx + 1, targets.size()) + "/" + targets.size()
                + ", heading " + Math.min(headingIdx + 1, headingCount) + "/" + headingCount
                + ", rows " + rows + ", dropped " + dropped
                + ", skippedSpots " + skippedSpots + ", skippedHeadings " + skippedHeadings
                + ", file " + filePath;
    }

    public void start() {
        if (isRunning()) return;

        state = State.IDLE;
        path = null;
        legActive = false;
        legNode = null;
        holdStarted = false;
        samplesThisHeading = 0;

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

        for (int k = 0; k < targets.size(); k++) {
            if (targets.get(k)[0] == nearest[0] && targets.get(k)[1] == nearest[1]) {
                Collections.rotate(targets, -k);
                break;
            }
        }

        headingCount = (int) Math.ceil(360.0 / Tunables.visionTunerHeadingStepDegrees);
        rows = 0;
        dropped = 0;
        skippedSpots = 0;
        skippedHeadings = 0;
        targetIdx = 0;
        headingIdx = 0;
        lastFrameTime = Double.NaN;

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
        if (isRunning()) path.addFirst(nearest);
    }

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

    public void update() {
        if (!isRunning()) return;
        switch (state) {
            case TRAVEL:
                updateTravel();
                break;
            case TURN:
                updateTurn();
                break;
            case SETTLE:
                if (stateTimer.milliseconds() >= Tunables.visionTunerSettleMillis) {
                    samplesThisHeading = 0;
                    state = State.SAMPLE;
                    stateTimer.reset();
                }
                break;
            case SAMPLE:
                updateSample();
                break;
            default:
                break;
        }
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

    private void beginTravel() {
        state = State.TRAVEL;
        legActive = false;
        legTimer.reset();
    }

    private void updateTravel() {
        if (legActive) {
            if (!follower.isBusy()) {
                curNode = legNode;
                legActive = false;
            } else if (legTimer.milliseconds() >= Tunables.visionTunerMoveTimeoutMillis) {
                abort("travel timeout");
                return;
            }
        }
        if (legActive) return;

        if (path.isEmpty()) {
            headingIdx = 0;
            beginHeading();
            return;
        }

        legNode = path.poll();
        Pose from = follower.pose();
        Pose to = new Pose(xs[legNode[0]], ys[legNode[1]], from.heading());
        if (Math.hypot(to.x() - from.x(), to.y() - from.y()) < MIN_LEG_LENGTH) {
            curNode = legNode;
            return;
        }
        follower.follow(line(from, to).linear(from, to)
                .with(Constants.foresightConfig.maxPathSpeed.at(Tunables.visionTunerSpeed)));
        legActive = true;
        legTimer.reset();
    }

    private void beginHeading() {
        state = State.TURN;
        holdStarted = false;
        stateTimer.reset();
    }

    private double targetHeading() {
        return AngleUnit.normalizeRadians(Math.toRadians(headingIdx * Tunables.visionTunerHeadingStepDegrees));
    }

    private void updateTurn() {
        double sx = xs[curNode[0]];
        double sy = ys[curNode[1]];
        double heading = targetHeading();
        if (!holdStarted) {
            follower.hold(new Pose(sx, sy, heading));
            holdStarted = true;
            stateTimer.reset();
        }
        Pose pose = follower.pose();
        double headingError = Math.abs(AngleUnit.normalizeRadians(pose.heading() - heading));
        boolean converged = headingError < Math.toRadians(Tunables.visionTunerHeadingToleranceDegrees)
                && Math.hypot(pose.x() - sx, pose.y() - sy) < Tunables.visionTunerPositionTolerance
                && follower.velocity().linSpeed() < 1;
        if (converged) {
            state = State.SETTLE;
            stateTimer.reset();
        } else if (stateTimer.milliseconds() >= Tunables.visionTunerMoveTimeoutMillis) {
            skippedHeadings++;
            nextHeading();
        }
    }

    private static double frameTime(LLResult result) {
        double timestamp = result.getTimestamp();
        return timestamp != 0 ? timestamp : result.getControlHubTimeStamp();
    }

    private void updateSample() {
        LLResult result = vision.getLatestResult();
        if (vision.isReasonable(result) && frameTime(result) != lastFrameTime) {
            lastFrameTime = frameTime(result);
            if (result.getBotposeTagCount() < Tunables.visionTunerMinTags) {
                dropped++;
            } else {
                try {
                    writeRow(result);
                } catch (IOException e) {
                    abort("CSV write failed");
                    return;
                }
                samplesThisHeading++;
            }
        }
        if (samplesThisHeading >= Tunables.visionTunerSamplesPerHeading
                || stateTimer.milliseconds() >= Tunables.visionTunerSampleTimeoutMillis) {
            try {
                writer.flush();
            } catch (IOException e) {
                abort("CSV write failed");
                return;
            }
            nextHeading();
        }
    }

    private void writeRow(LLResult result) throws IOException {
        Pose pose = follower.pose();
        Position botpose = result.getBotpose().getPosition().toUnit(DistanceUnit.INCH);
        double yaw = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS);
        writer.write(String.format(Locale.US, "%.4f,%.4f,%.5f,%.4f,%.4f,%.4f,%.4f,%.5f,%d",
                pose.x(), pose.y(), AngleUnit.normalizeRadians(pose.heading()), Tunables.visionTunerTrueZ,
                botpose.y + 72, 72 - botpose.x, botpose.z, AngleUnit.normalizeRadians(yaw - Math.PI / 2),
                result.getBotposeTagCount()));
        writer.newLine();
        rows++;
    }

    private void nextHeading() {
        headingIdx++;
        if (headingIdx < headingCount) {
            beginHeading();
            return;
        }
        targetIdx++;
        advanceTarget();
    }

    private void advanceTarget() {
        headingIdx = 0;
        while (targetIdx < targets.size()) {
            ArrayDeque<int[]> route = bfs(curNode, targets.get(targetIdx));
            if (route != null) {
                path = route;
                beginTravel();
                return;
            }
            skippedSpots++;
            targetIdx++;
        }
        finish();
    }

    private void buildGrid() {
        xs = axis(Tunables.visionTunerSpotX);
        ys = axis(Tunables.visionTunerSpotY);
        valid = new boolean[xs.length][ys.length];
        for (int i = 0; i < xs.length; i++) {
            for (int j = 0; j < ys.length; j++) {
                valid[i][j] = isPointSafe(xs[i], ys[j]);
            }
        }
    }

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

    private boolean isPointSafe(double x, double y) {
        if (x < clearance - EPSILON || x > FIELD_SIZE - clearance + EPSILON
                || y < clearance - EPSILON || y > FIELD_SIZE - clearance + EPSILON) return false;
        return rectDistance(x, y, Tunables.visionTunerObstacleAMinX, Tunables.visionTunerObstacleAMinY,
                Tunables.visionTunerObstacleAMaxX, Tunables.visionTunerObstacleAMaxY) >= clearance - EPSILON
                && rectDistance(x, y, Tunables.visionTunerObstacleBMinX, Tunables.visionTunerObstacleBMinY,
                Tunables.visionTunerObstacleBMaxX, Tunables.visionTunerObstacleBMaxY) >= clearance - EPSILON;
    }

    private boolean isSegmentSafe(double x0, double y0, double x1, double y1) {
        double length = Math.hypot(x1 - x0, y1 - y0);
        int steps = Math.max(1, (int) Math.ceil(length / SEGMENT_STEP));
        for (int s = 0; s <= steps; s++) {
            double t = (double) s / steps;
            if (!isPointSafe(x0 + (x1 - x0) * t, y0 + (y1 - y0) * t)) return false;
        }
        return true;
    }

    private static double rectDistance(double x, double y, double minX, double minY, double maxX, double maxY) {
        double dx = Math.max(Math.max(minX - x, 0), x - maxX);
        double dy = Math.max(Math.max(minY - y, 0), y - maxY);
        return Math.hypot(dx, dy);
    }

    private int[] nearestNode(double x, double y) {
        int[] best = null;
        double bestDist = Double.MAX_VALUE;
        for (int i = 0; i < xs.length; i++) {
            for (int j = 0; j < ys.length; j++) {
                if (!valid[i][j]) continue;
                double d = Math.hypot(xs[i] - x, ys[j] - y);
                if (d < bestDist && isSegmentSafe(x, y, xs[i], ys[j])) {
                    bestDist = d;
                    best = new int[]{i, j};
                }
            }
        }
        return best;
    }

    private List<int[]> serpentine() {
        List<int[]> out = new ArrayList<>();
        for (int j = 0; j < ys.length; j++) {
            boolean forward = j % 2 == 0;
            for (int k = 0; k < xs.length; k++) {
                int i = forward ? k : xs.length - 1 - k;
                if (valid[i][j]) out.add(new int[]{i, j});
            }
        }
        return out;
    }

    private boolean edgeSafe(int i0, int j0, int i1, int j1) {
        return valid[i1][j1] && isSegmentSafe(xs[i0], ys[j0], xs[i1], ys[j1]);
    }

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
