package org.firstinspires.ftc.teamcode.Processors.Vision;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ColorSpace;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.List;

/**
 * Color blob localizer: wraps the camera and the ColorBlobLocatorProcessor internally
 * and exposes a live result interface.
 *
 * Usage:
 *   ColorLocator locator = new ColorLocator(hardwareMap);
 *   while (opModeIsActive()) {
 *       if (locator.update()) {
 *           double x = locator.getCenterX();  // normalized coordinates
 *       }
 *   }
 */
public class ColorLocator {

    // Camera resolution (downsampled; must match the VisionPortal settings)
    private static final int CAMERA_WIDTH = 640;
    private static final int CAMERA_HEIGHT = 360;

    // Camera and processors
    private final List<ColorBlobLocatorProcessor> processors = new ArrayList<>();
    private VisionPortal portal;

    // Latest results (updated after each update() call)
    private boolean targetFound = false;
    private double centerX = 0.0;   // normalized [-1, 1], positive to the right
    private double centerY = 0.0;   // normalized [-1, 1], positive upward
    private double boxWidth = 0.0;
    private double boxHeight = 0.0;
    private double boxAngle = 0.0;
    private double radius = 0.0;

    public ColorLocator(HardwareMap hardwareMap) {
        this(hardwareMap, "Webcam 1");
    }

    public ColorLocator(HardwareMap hardwareMap, String webcamName) {
        // Build one processor per color
        ColorBlobLocatorProcessor proc0 = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
                                    new Scalar(0, 0, 0),
                                    new Scalar(255, 255, 255)))
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setBlurSize(5)
                .setErodeSize(0)
                .setDilateSize(0)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
        processors.add(proc0);

        // Wrap the camera: attach all processors to the same VisionPortal
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .addProcessor(proc0)
                .setCameraResolution(new Size(CAMERA_WIDTH, CAMERA_HEIGHT))
                .setCamera(hardwareMap.get(WebcamName.class, webcamName));
        portal = builder.build();
    }

    /**
     * Read the latest frame and locate the target color blob; return whether one was found.
     * The fitted contour center is normalized to [-1, 1], positive to the upper right:
     * x grows to the right, y grows upward, and the frame center is (0, 0).
     */
    public boolean update() {
        List<ColorBlobLocatorProcessor.Blob> allBlobs = new ArrayList<>();

        // Processor 0: read blobs and filter by rule
        List<ColorBlobLocatorProcessor.Blob> blobs0 = processors.get(0).getBlobs();
        allBlobs.addAll(blobs0);

        // Global sort
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, allBlobs);

        if (allBlobs.isEmpty()) {
            targetFound = false;
            centerX = 0.0;
            centerY = 0.0;
            return false;
        }

        targetFound = true;
        ColorBlobLocatorProcessor.Blob first = allBlobs.get(0);
        Circle circle = first.getCircle();
        // Normalize to [-1, 1]: positive x to the right, positive y upward
        centerX = (circle.getX() / CAMERA_WIDTH) * 2.0 - 1.0;
        centerY = 1.0 - (circle.getY() / CAMERA_HEIGHT) * 2.0;
        radius = circle.getRadius();
        return true;
    }

    public boolean isTargetFound() { return targetFound; }
    public double getCenterX() { return centerX; }
    public double getCenterY() { return centerY; }
    public double getRadius() { return radius; }

    public void close() {
        if (portal != null) {
            portal.close();
        }
    }
}
