# Official ColorLocator Usage Guide

ColorLocator is built on the FTC SDK's built-in `ColorBlobLocatorProcessor` (hereafter CBLP), which is
driven by a `VisionPortal` to find "blobs" in the camera image that fall inside a specified color range, and returns each blob's
contour, fitted rectangle/circle, area, circularity, and other geometric information.

> Difference from a color sensor (`ColorSensor`): a ColorSensor can only tell the color of an object
> placed close to the sensor, while ColorLocator **locates** the position and shape of target color blobs
> across the whole (or part of) the image.

Reference official samples:

- `ConceptVisionColorLocator_Rectangle.java` (rectangle fit)
- `ConceptVisionColorLocator_Circle.java` (circle fit)

***

## 1. Main Dependencies (imports)

```java
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.SortOrder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.Circle;        // Circle fit (Circle sample)
import org.firstinspires.ftc.vision.opencv.ColorSpace;     // Used for custom color ranges

import org.opencv.core.RotatedRect;  // Rectangle fit
import org.opencv.core.Scalar;       // Used for custom color ranges

import java.util.List;
```

***

## 2. Workflow

1. **Build the processor**: configure the target color range, ROI, contour mode,
   preprocessing (blur/erode/dilate), etc. with `ColorBlobLocatorProcessor.Builder`; `build()` produces `colorLocator`.
2. **Build the VisionPortal**: attach `colorLocator` with `addProcessor`, and set the resolution and camera.
3. **Read results**: call `colorLocator.getBlobs()` in the loop to get a `List<Blob>`.
4. **Filter/sort**: use `ColorBlobLocatorProcessor.Util` to filter and sort by area, circularity, etc.
5. From each `Blob` you can read the center coordinates, area, density, aspect ratio, perimeter, circularity, and the rectangle/circle fit.

Internally: pixels matching the `ColorRange` form a binary `mask` -> grouped into connected `blob`s -> contours
`contour` are extracted -> for each blob, the smallest rectangle `boxFit` fully enclosing the contour is generated (or the smallest circle).

***

## 3. Building the Processor and Parameter Reference

```java
ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Target color range (required)
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .setRoi(ImageRegion.asUnityCenterCoordinates(-0.75, 0.75, 0.75, -0.75))
        .setDrawContours(true)                              // Whether to draw contours on the preview
        .setBoxFitColor(0)                                  // Rectangle-fit color, 0 disables
        .setCircleFitColor(Color.rgb(255, 255, 0))          // Circle-fit color, 0 disables
        .setBlurSize(5)                                     // Blur
        .setDilateSize(15)                                  // Dilate
        .setErodeSize(15)                                   // Erode
        .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
        .build();
```

### 3.1 setTargetColorRange(ColorRange) -- Target Color Range

**Predefined colors** (simplest):

| Constant                                         | Description        |
| ---------------------------------------------- | ----- |
| `ColorRange.RED` / `BLUE` / `YELLOW` / `GREEN` | Basic colors |
| `ColorRange.ARTIFACT_GREEN`                    | Green game element |
| `ColorRange.ARTIFACT_PURPLE`                   | Purple game element |

**Custom color range** (give lower/upper bounds in a color space such as `YCrCb`):

```java
.setTargetColorRange(new ColorRange(ColorSpace.YCrCb,
        new Scalar( 32, 176,   0),   // Lower bound
        new Scalar(255, 255, 132)))  // Upper bound
```

### 3.1.1 Officially Supported Color Spaces

The official `org.firstinspires.ftc.vision.opencv.ColorSpace` enum provides only the following **three** color spaces,
used to declare which space the lower/upper bound values belong to when defining a custom `ColorRange`:

| Enum value | Meaning | OpenCV conversion code (BGR to that space) | Per-channel range (8-bit image) |
| ------- | ------------ | ------------------------- | ------------------------------------ |
| `RGB`   | Red-Green-Blue | `Imgproc.COLOR_BGR2RGB`   | R / G / B: 0 \~ 255                   |
| `HSV`   | Hue-Saturation-Value | `Imgproc.COLOR_BGR2HSV`   | H: 0 \~ 180 (see below), S / V: 0 \~ 255       |
| `YCrCb` | Luma-RedChroma-BlueChroma | `Imgproc.COLOR_BGR2YCrCb` | Y / Cr / Cb: 0 \~ 255 (Cr and Cb centered near 128) |

> Notes:
>
> - The official `ColorLocator` / `ColorBlobLocatorProcessor` does **not support Lab, HSL/HLS, or other spaces**.
> - In OpenCV, the HSV **H channel is actually 0 \~ 179** (written as 0 \~ 180 in the official sample comments),
>   while S and V are both 0 \~ 255.
> - As seen in the sample source, the predefined color constants (`ColorRange.RED` / `BLUE` / `YELLOW` / `GREEN` /
>   `ARTIFACT_GREEN` / `ARTIFACT_PURPLE`) all use bounds calibrated in the `YCrCb` space internally.

### 3.2 setRoi(ImageRegion) -- Region of Interest (optional)

Searching only part of the image reduces distractions and improves speed. `ImageRegion` has three forms:

```java
ImageRegion.entireFrame();                                    // Whole frame
ImageRegion.asImageCoordinates(50, 50, 150, 150);             // Pixel coordinates: 100x100 starting at top-left
ImageRegion.asUnityCenterCoordinates(-0.5, 0.5, 0.5, -0.5);   // Normalized coordinates (-1..1, origin at center)
```

> Note that in `asUnityCenterCoordinates(uMin, vMin, uMax, vMax)` the v direction is opposite to image y,
> with positive values upward; the sample `(-0.75, 0.75, 0.75, -0.75)` covers the central 75% width/height of the image.

### 3.3 setContourMode(ContourMode) -- Contour Mode

| Mode | Description |
| ------------------------- | -------------------------------- |
| `EXTERNAL_ONLY`           | Keep only outermost contours, **recommended**; avoids bright-spot reflections splitting a solid blob into fragments |
| `ALL_FLATTENED_HIERARCHY` | Keep all contours (including nested ones) |

### 3.4 Preprocessing (optional morphology/denoising)

| Method | Description |
| ---------------------------- | -------------------------------------------------------- |
| `setBlurSize(int px)`        | Gaussian blur, smooths color transitions and contours; larger values blur more and hide small features. 5 is suggested for low-resolution images. Even values are automatically bumped by 1 to meet the odd-number requirement |
| `setErodeSize(int px)`       | Erosion: removes isolated pixels and thin lines, shrinks objects, and may enlarge interior holes. 2\~4 is suggested at low resolution |
| `setDilateSize(int px)`      | Dilation: fills small holes, joins broken parts, and enlarges objects. 2\~4 is suggested at low resolution |
| `setMorphOperationType(...)` | Erode/dilate order: `OPENING` = erode then dilate (removes small noise); `CLOSING` = dilate then erode (fills edge gaps) |

### 3.5 Preview Overlay Drawing (for debugging, costs CPU)

| Method | Description |
| ------------------------------ | -------------------------------------- |
| `setDrawContours(boolean)`     | Whether to draw contours on the preview |
| `setBoxFitColor(int color)`    | Rectangle-fit color (`android.graphics.Color`), `0` disables |
| `setCircleFitColor(int color)` | Circle-fit color, `0` disables |

***

## 4. Building the VisionPortal

```java
VisionPortal portal = new VisionPortal.Builder()
        .addProcessor(colorLocator)
        .setCameraResolution(new Size(320, 240))   // Low resolution is enough here; improves performance and lowers latency
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Or use BuiltinCameraDirection.BACK
        .build();
```

***

## 5. Reading Results: the Blob Object

```java
List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
```

Blobs are returned in **descending** contour-area order by default (largest blob first). Each `Blob` provides:

| Method | Returns | Description |
| ------------------ | ------------- | ---------------------------------------- |
| `getBoxFit()`      | `RotatedRect` | Smallest rotated rectangle enclosing the contour; `boxFit.center.x/.y` are the center pixel coordinates |
| `getCircle()`      | `Circle`      | Smallest circle enclosing the contour; `getX()/getY()/getRadius()` |
| `getContourArea()` | int           | Pixel area inside the contour |
| `getDensity()`     | double        | Fill = contour area / convex-hull area; closer to 1 means more "solid" |
| `getAspectRatio()` | double        | Long side / short side of the fit box; a square is about 1 |
| `getArcLength()`   | double        | Contour perimeter |
| `getCircularity()` | double        | Circularity; a perfect circle is about 1, smaller means less circular |

***

## 6. Filtering and Sorting

```java
// Filter: keep only blobs with area 50-20000
ColorBlobLocatorProcessor.Util.filterByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, blobs);

// Filter: keep only blobs with circularity 0.6-1 (circular targets)
ColorBlobLocatorProcessor.Util.filterByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY, 0.6, 1, blobs);

// Sort (call at most once)
ColorBlobLocatorProcessor.Util.sortByCriteria(
        ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, blobs);
```

Available `BlobCriteria`:

| Enum | Description |
| ----------------- | --- |
| `BY_CONTOUR_AREA` | Area |
| `BY_DENSITY`      | Density |
| `BY_ASPECT_RATIO` | Aspect ratio |
| `BY_ARC_LENGTH`   | Perimeter |
| `BY_CIRCULARITY`  | Circularity |

> `filterByCriteria` only affects the returned `blobs` list; it does not change the contours drawn on the preview overlay.

***

## 7. Multi-Color Search (Multiple Processors on One VisionPortal)

`setTargetColorRange(ColorRange)` accepts only **one** color range at a time, and a single `ColorBlobLocatorProcessor`
cannot directly express a "union over multiple colors". To search for several colors at once (e.g. red and blue), the approach is: **build one
`ColorBlobLocatorProcessor` per color, attach all of them with** **`addProcessor(...)`** **to the same** **`VisionPortal`,
then merge their** **`getBlobs()`** **results manually**.

Benefit: each processor can have its own `setRoi` / `setContourMode` / preprocessing parameters, and the covered regions may overlap.

```java
import java.util.ArrayList;
import java.util.List;

// Build one processor per color
ColorBlobLocatorProcessor redLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.RED)
        .setRoi(ImageRegion.entireFrame())
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .build();

ColorBlobLocatorProcessor blueLocator = new ColorBlobLocatorProcessor.Builder()
        .setTargetColorRange(ColorRange.BLUE)
        .setRoi(ImageRegion.entireFrame())
        .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
        .build();

// Attach to the same VisionPortal
VisionPortal portal = new VisionPortal.Builder()
        .addProcessor(redLocator)
        .addProcessor(blueLocator)
        .setCameraResolution(new Size(320, 240))
        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
        .build();

while (opModeIsActive() || opModeInInit()) {
    // Union: merge results from each processor
    List<ColorBlobLocatorProcessor.Blob> all = new ArrayList<>();
    all.addAll(redLocator.getBlobs());
    all.addAll(blueLocator.getBlobs());

    // Filter/sort uniformly after merging
    ColorBlobLocatorProcessor.Util.filterByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, 50, 20000, all);
    ColorBlobLocatorProcessor.Util.sortByCriteria(
            ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, all);

    for (ColorBlobLocatorProcessor.Blob b : all) {
        RotatedRect box = b.getBoxFit();
        telemetry.addData("Center", "(%3.0f, %3.0f)", box.center.x, box.center.y);
    }
    telemetry.update();
    sleep(100);
}
```

Notes:

- For a wide range of "one hue wrapping around 0 degrees" only (e.g. HSV red with H in \[170,180] union \[0,10]),
  a single rectangular `ColorRange` interval cannot cover it; two processors built with this same approach work;
- For a more flexible "union over arbitrary intervals" (e.g. custom color spaces, merging several bounds at once), switch to a custom
  `VisionProcessor` that applies multi-interval `inRange` + `bitwise_or` directly on the mask (see this project's `ColorSegCam`).

### 7.1 Merging Blobs of Different Colors (Treating a Multi-Color Composite as One Blob)

If the goal is "one object pieced together from red, green, and **other colors**, recognized as a single blob", the official CBLP cannot merge
at the pixel level: each processor thresholds independently and `Blob` does not expose the raw mask/contour, so the two colors only produce
two unrelated blobs. There are two ways to merge them into one:

**Option 1: Approximate geometric merge (using only the official `Blob` geometry)**

For the merged `all` list from Section 7, merge neighboring blobs whose bounding boxes overlap/are adjacent: area-weighted centroid,
summed area, and union of bounding boxes.

```java
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;

// Simple "composite blob" carrying the merged geometry
static class MergedBlob {
    Rect bounds;   // Bounding box (AABB approximation)
    double cx, cy; // Area-weighted centroid (pixel coordinates)
    int area;      // Summed area
}

List<MergedBlob> merged = new ArrayList<>();
for (ColorBlobLocatorProcessor.Blob b : all) {
    RotatedRect box = b.getBoxFit();
    Rect r = box.boundingRect();
    MergedBlob hit = null;
    for (MergedBlob m : merged) {
        // Whether the two bounding boxes overlap, or the gap is <= 10 pixels
        boolean overlap = m.bounds.x < r.x + r.width + 10 && r.x < m.bounds.x + m.bounds.width + 10
                       && m.bounds.y < r.y + r.height + 10 && r.y < m.bounds.y + m.bounds.height + 10;
        if (overlap) { hit = m; break; }
    }
    if (hit == null) {
        MergedBlob m = new MergedBlob();
        m.bounds = new Rect(r.x, r.y, r.width, r.height);
        m.cx = box.center.x;
        m.cy = box.center.y;
        m.area = b.getContourArea();
        merged.add(m);
    } else {
        int a = b.getContourArea();
        double na = hit.area + a;                                    // Sum areas
        hit.cx = (hit.cx * hit.area + box.center.x * a) / na;        // Area-weighted centroid
        hit.cy = (hit.cy * hit.area + box.center.y * a) / na;
        hit.area = (int) na;
        int x1 = Math.min(hit.bounds.x, r.x);                        // Union of bounding boxes
        int y1 = Math.min(hit.bounds.y, r.y);
        int x2 = Math.max(hit.bounds.x + hit.bounds.width,  r.x + r.width);
        int y2 = Math.max(hit.bounds.y + hit.bounds.height, r.y + r.height);
        hit.bounds = new Rect(x1, y1, x2 - x1, y2 - y1);
    }
}
```

> Limitation: this merges purely by geometric proximity and does not know whether the underlying pixels are truly connected. Two
> **same-color objects that merely touch** would also be merged by mistake; only **differently colored yet physically connected**
> objects merge correctly, so this suits scenarios where accuracy demands are modest.

**Option 2: Pixel-level merge (custom `VisionProcessor`, recommended)**

To truly connect "adjacent pixels of different colors into the same blob", all target colors must land in the **same mask**: run several
`inRange` intervals on the same frame, then merge them into one mask with `bitwise_or`, and finally run connected components / contour finding. This is exactly
the approach used by this project's `ColorSegCam`; each official CBLP processor owns a separate mask and cannot do this.

***

## 8. Complete Code Sample


```java
        // ===== Required imports (fill in per project template) =====
        // import java.util.ArrayList;
        // import java.util.List;
        // import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        // import org.firstinspires.ftc.robotcore.external.navigation.Size;
        // import org.firstinspires.ftc.robotcore.external.tfod.SortOrder;
        // import org.firstinspires.ftc.vision.VisionPortal;
        // import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
        // import org.firstinspires.ftc.vision.opencv.ColorRange;
        // import org.firstinspires.ftc.vision.opencv.ColorSpace;
        // import org.firstinspires.ftc.vision.opencv.ImageRegion;
        // import org.opencv.core.RotatedRect;

        // ===== Init phase =====  (place at the start of runOpMode)
        // Build each ColorBlobLocatorProcessor
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

        // Attach to the same VisionPortal
        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(proc0)
                .setCameraResolution(new Size(640, 360))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))  // Or use BuiltinCameraDirection.BACK
                .build();

        telemetry.setMsTransmissionInterval(100);

        // ===== Run phase =====  (place inside the runOpMode loop)
        // Read the blob list from each processor
        List<ColorBlobLocatorProcessor.Blob> blobs0 = proc0.getBlobs();

        // Merge all blobs
        List<ColorBlobLocatorProcessor.Blob> allBlobs = new ArrayList<>();
        allBlobs.addAll(blobs0);

        // Global sort
        ColorBlobLocatorProcessor.Util.sortByCriteria(ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA, SortOrder.DESCENDING, allBlobs);

        // Take only the top-ranked blob, and store its fitted shape plus key parameters
        if (!allBlobs.isEmpty()) {
            ColorBlobLocatorProcessor.Blob firstBlob = allBlobs.get(0);
            ColorBlobLocatorProcessor.Circle circle = firstBlob.getCircle();
            double circleCenterX = circle.getX();
            double circleCenterY = circle.getY();
            double circleRadius = circle.getRadius();
        }

```

***

## 9. Debugging and Performance Tips

- **Live preview**: write `while (opModeIsActive())` as `while (opModeIsActive() || opModeInInit())`,
  so you can see the camera stream and blob boxes during the INIT phase, which helps calibrate parameters in the field.
- **Viewing the image**: connect an HDMI monitor to the Control Hub, or mirror with ScrCpy.
- **Resolution**: ColorLocator is insensitive to resolution; `320x240` is enough, saving CPU and reducing latency.
- **Lighting affects circularity**: shadows change `circularity`; if the rules allow calibration, measure thresholds on the actual field.
- **CPU cost**: drawing via `setDrawContours`/`setBoxFitColor`/`setCircleFitColor` costs CPU,
  so you can turn it off in the final match code once debugging is done.

