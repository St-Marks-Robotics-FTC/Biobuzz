# This Approach Is Deprecated

The following is the implementation plan for uploading an AprilTag map to the Limelight 3A, covering the REST API specification, Java code examples, and integration with the HIVE state machine.

***

## 1. Interface Specification

Limelight OS runs a REST/HTTP server on `(limelight-ip):5807`. The endpoint for uploading a field map is:

| Item             | Content                                           |
| :--------------- | :------------------------------------------------ |
| **Method**       | `POST`                                            |
| **Route**        | `/upload-fieldmap`                                |
| **Request body** | `.fmap` data in JSON format                       |
| **Optional arg** | URL parameter `index` (loads into the given pipeline; defaults to the current pipeline) |
| **Full URL**     | `http://<limelight-ip>:5807/upload-fieldmap`      |

After upload, the map overwrites the field map on disk, and Limelight immediately uses the new map for MegaTag pose estimation.

### .fmap File Format

An `.fmap` file is a JSON file containing a `fiducials` array, where each entry describes one AprilTag:

```json
{
  "type": "ftc",
  "fiducials": [
    {
      "family": "36h11",
      "id": 30,
      "size": 82.5,
      "transform": [
        1, 0, 0, -36.0,
        0, 1, 0,  22.0,
        0, 0, 1,  -15.0,
        0, 0, 0,   1.0
      ],
      "unique": 1
    }
  ]
}
```

| Field       | Description                                                             |
| :---------- | :---------------------------------------------------------------------- |
| `type`      | Field type; FTC uses `"ftc"` or `"ftcd"` (diamond field)                |
| `family`    | AprilTag family; BIOBUZZ uses `36h11`                                   |
| `id`        | Tag ID (30-45)                                                          |
| `size`      | Tag size in millimeters; BIOBUZZ uses 82.5 mm (3.25 in)                 |
| `transform` | 4x4 row-major transform matrix, SI units (meters), pose of this tag in the field frame |
| `unique`    | Whether unique (1 = this ID appears only once)                          |

Before a match you would generate two `.fmap` files, one for each stable HIVE state. Each file contains only the 4 tags of the currently **face-up CELL**. The transform matrices must be computed from the official BIOBUZZ HIVE geometry and the tag positions in each stable state.

## 2. Java Upload Implementation

The FTC Android environment has no default dependency on third-party HTTP libraries such as `OkHttp`; the JDK built-in `HttpURLConnection` is sufficient. The following is a utility class that can be integrated directly into `MT1Localizer`:

```java
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.nio.charset.StandardCharsets;

public class LimelightMapUploader {

    /** Limelight default IP (when connected over USB) */
    private static final String LIMELIGHT_IP = "172.28.0.1";
    private static final int PORT = 5807;

    /**
     * Upload an .fmap file to Limelight.
     *
     * @param fmapJson  JSON string content of the .fmap file
     * @param pipelineIndex  target pipeline index (-1 means use the current pipeline)
     * @return true if the upload succeeded (HTTP 200)
     */
    public static boolean uploadFieldMap(String fmapJson, int pipelineIndex) {
        HttpURLConnection conn = null;
        try {
            String urlStr = "http://" + LIMELIGHT_IP + ":" + PORT + "/upload-fieldmap";
            if (pipelineIndex >= 0) {
                urlStr += "?index=" + pipelineIndex;
            }

            URL url = new URL(urlStr);
            conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setRequestProperty("Content-Type", "application/json");
            conn.setDoOutput(true);
            conn.setConnectTimeout(2000);
            conn.setReadTimeout(3000);

            // Write the JSON request body
            try (OutputStream os = conn.getOutputStream()) {
                byte[] input = fmapJson.getBytes(StandardCharsets.UTF_8);
                os.write(input, 0, input.length);
            }

            int responseCode = conn.getResponseCode();
            return responseCode == HttpURLConnection.HTTP_OK;

        } catch (Exception e) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }

    /**
     * Switch pipeline (no need to re-upload the map).
     */
    public static boolean switchPipeline(int pipelineIndex) {
        HttpURLConnection conn = null;
        try {
            String urlStr = "http://" + LIMELIGHT_IP + ":" + PORT
                    + "/pipeline-switch?index=" + pipelineIndex;
            URL url = new URL(urlStr);
            conn = (HttpURLConnection) url.openConnection();
            conn.setRequestMethod("POST");
            conn.setConnectTimeout(1000);
            conn.setReadTimeout(1000);
            return conn.getResponseCode() == HttpURLConnection.HTTP_OK;
        } catch (Exception e) {
            return false;
        } finally {
            if (conn != null) conn.disconnect();
        }
    }
}
```

**Key notes**:

- The default IP of Limelight connected to the Control Hub over USB is `172.28.0.1`. Adjust it if you connect over Wi-Fi or have changed the IP.
- The `pipeline-switch` endpoint likewise accepts the URL parameter `index`, with extremely low switching latency (milliseconds).
- Upload and switch operations must run **on a background thread** to avoid blocking the OpMode main loop. Use `new Thread(() -> {...}).start()`.

## Notes

1. **Confirm the IP address**: the IP is `172.28.0.1` when Limelight is connected to the Control Hub over USB. For a Wi-Fi connection, check the actual IP in the Limelight web UI.
2. **Thread safety**: `uploadFieldMap` and `switchPipeline` must run on a background thread and must not block the OpMode `loop()`.
3. **Preload pipelines**: it is recommended to upload the two maps to Pipeline 0 and Pipeline 1 respectively via the web UI before a match. During a match only `pipelineSwitch` is needed, whose millisecond-level latency is much faster than re-uploading a map each time.
4. **fmap coordinate system**: the Limelight fmap uses the field center as the origin. Field coordinates provided by the official BIOBUZZ documents may need conversion. Keep units in meters when generating the transform matrices.
5. **Upload failure fallback**: if `switchPipeline` returns `false`, keep the current map unchanged and consider falling back to manual mode, where the operator selects the state via the Driver Station.
6. **Tag ID filtering linked with map switching**: after switching pipelines, the `SetFiducialIDFiltersOverride` filter list should be updated together so that only the 4 tag IDs of the currently face-up CELL are kept.
7. **Availability of `LimelightHelpers`**: `LimelightHelpers` is not shipped with the FTC SDK; copy the `LimelightHelpers.java` file into the project. If you prefer not to bring in that file, `SetFiducialIDFiltersOverride` can also be implemented by sending JSON to the REST API `POST /update-pipeline`.
