package org.hivevisionftc.vision.VisionDrivers;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

import org.hivevisionftc.vision.Detection;
import org.hivevisionftc.vision.Pose;
import org.hivevisionftc.vision.VisionConfig;

import java.util.ArrayList;

/**
 * Webcam track for the core library.
 *
 * <p>Runs the official Hive Vision repo's Control Hub detector
 * ({@link BallDetectorPipeline}: OpenCV HSV color blobs through a
 * {@link VisionPortal}, no model file) and adapts its blobs to the same
 * {@link Detection} list the Limelight track returns, so everything above this
 * class - {@code HiveVision}, the engine adapter, the planner - cannot tell
 * which camera produced a frame. Blob colors map onto the shared class IDs
 * (0 = pollen, 1 = red nectar, 2 = blue nectar, see {@code Classifier}).
 *
 * <p>The pipeline gates blob size in pixels at 1280x720, which is why this
 * driver fixes that resolution, same as the official OpMode. Angles come from
 * the blob center and the camera FOV the caller passes in; measure your
 * camera's FOV rather than trusting the adapter's defaults.
 */
public class WebcamDriver {
    private final BallDetectorPipeline pipeline = new BallDetectorPipeline();
    private final VisionPortal portal;
    private final VisionConfig visionConfig;
    private final double hFovDeg;
    private final double vFovDeg;

    public WebcamDriver(WebcamName webcam, VisionConfig visionConfig,
                        double hFovDeg, double vFovDeg) {
        this.visionConfig = visionConfig;
        this.hFovDeg = hFovDeg;
        this.vFovDeg = vFovDeg;
        this.portal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(1280, 720))
                .addProcessor(pipeline)
                .enableLiveView(true)
                .build();
    }

    public ArrayList<Detection> fetchLatestDetections(Pose robotPose) {
        ArrayList<Detection> detections = new ArrayList<>();
        for (BallBlobPipeline.BallBlob blob : pipeline.getDetections()) {
            detections.add(toDetection(blob, robotPose, visionConfig, hFovDeg, vFovDeg));
        }
        return detections;
    }

    /** One blob as a Detection: center angles from the FOV, fill ratio as confidence. */
    static Detection toDetection(BallBlobPipeline.BallBlob blob, Pose robotPose,
                                 VisionConfig cfg, double hFovDeg, double vFovDeg) {
        double tx = (blob.cxNorm - 0.5) * hFovDeg;    // + = right of center
        double ty = (0.5 - blob.cyNorm) * vFovDeg;    // + = above center
        double areaPct = blob.wNorm * blob.hNorm * 100.0;
        double fill = blob.rect.width > 0 && blob.rect.height > 0
                ? blob.area / (blob.rect.width * (double) blob.rect.height) : 0;
        Detection d = new Detection(robotPose, cfg, tx, ty,
                Math.max(0.05, Math.min(0.95, fill)), areaPct);
        d.setClassId(classId(blob.color));
        return d;
    }

    /** Must match Classifier.BiobuzzDetections: 0 pollen, 1 red nectar, 2 blue nectar. */
    private static int classId(BallBlobPipeline.BallColor color) {
        switch (color) {
            case RED: return 1;
            case BLUE: return 2;
            default: return 0;
        }
    }

    /** Milliseconds since the pipeline last processed a frame, or -1 before the first. */
    public long frameAgeMs() {
        long nanos = pipeline.lastCaptureNanos();
        return nanos < 0 ? -1 : (System.nanoTime() - nanos) / 1_000_000;
    }

    /** Capture timestamp of the latest frame; changes only when a new frame lands. */
    public long lastCaptureNanos() {
        return pipeline.lastCaptureNanos();
    }

    public void close() {
        portal.close();
    }
}
