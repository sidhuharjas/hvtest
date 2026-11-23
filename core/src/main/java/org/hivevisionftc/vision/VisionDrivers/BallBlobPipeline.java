/*
 * BallBlobPipeline - shared base for the two Control Hub ball detectors
 * (cv/TeamCode/BallDetectorPipeline.java and archive/lab/TeamCode/LabBallDetectorPipeline.java).
 * Vendored from the official Hive Vision repo (hivevisionftc/Hive-Vision),
 * package moved under org.hivevisionftc.vision.VisionDrivers.
 * Everything identical between them lives here, so a fix (bestOf()'s cost
 * shape, a new field on BallBlob) can't silently land in only one detector:
 *   - BallColor + BallBlob
 *   - bestOf() / getDetections() candidate API
 *   - suspend / resume / suspendDetection / resumeDetection control
 *   - the VisionProcessor loop contract: processFrame() gating, blob
 *     normalization, snapshot hand-off, and the debug overlay
 *   - contour -> shape/size gate -> blob extraction (blobsFromMask)
 *
 * Subclasses implement only their per-color math:
 *   expectArea(BallColor)  - "typical real-ball area" per color
 *   findBlobs(input, out)  - threshold the frame, call blobsFromMask() per
 *                            color, append BallBlobs to `out`
 *
 * Threading: `suspend`/`detect` are written by the OpMode thread and read by
 * the VisionProcessor thread every frame, so they are volatile. `detections`
 * is swapped under `lock` and only read through bestOf()/getDetections().
 */
package org.hivevisionftc.vision.VisionDrivers;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;

public abstract class BallBlobPipeline implements VisionProcessor {

    public enum BallColor { YELLOW, RED, BLUE }

    /** One detected ball candidate for the OpMode. */
    public static class BallBlob {
        public final BallColor color;
        public final Rect   rect;
        public final double area;
        public final double cx, cy;
        public double cxNorm = 0, cyNorm = 0, wNorm = 0, hNorm = 0;

        BallBlob(BallColor c, Rect r, double a) {
            color = c;
            rect = r;
            area = a;
            cx = r.x + r.width / 2.0;
            cy = r.y + r.height / 2.0;
        }

        void normalize(int frameW, int frameH) {
            cxNorm = cx / frameW;
            cyNorm = cy / frameH;
            wNorm = rect.width / (double) frameW;
            hNorm = rect.height / (double) frameH;
        }
    }

    protected static final boolean USE_MORPH = true;    // close small gaps

    // Added for the Hive Engine integration: the capture timestamp of the most
    // recent processed frame, so callers can run the same staleness gate the
    // Limelight track uses. Everything else in this file is upstream.
    private volatile long lastCaptureNanos = -1;

    /** Capture timestamp (nanos) of the latest processed frame, or -1 before the first. */
    public long lastCaptureNanos() { return lastCaptureNanos; }

    private volatile boolean suspend = false;
    private volatile boolean detect  = true;
    private final Object lock = new Object();
    private List<BallBlob> detections = new ArrayList<>();

    /** Typical real-ball contour area per color (px at camera resolution). */
    protected abstract double expectArea(BallColor c);

    /** Extract this track's color candidates into `out` (only called while detection is active). */
    protected abstract void findBlobs(Mat input, List<BallBlob> out);

    /** Most ball-like detected blob of a color (candidate, not ground truth). */
    public BallBlob bestOf(BallColor color) {
        BallBlob best = null;
        double bestCost = Double.MAX_VALUE;
        synchronized (lock) {
            for (BallBlob b : detections) {
                if (b.color != color) continue;
                double fill = b.area / (double) (b.rect.width * b.rect.height);
                double cost = Math.abs(b.area / expectArea(color) - 1.0)
                            + 4.0 * (1.0 - fill);
                if (cost < bestCost) {
                    bestCost = cost;
                    best = b;
                }
            }
        }
        return best;
    }

    public List<BallBlob> getDetections() {
        synchronized (lock) {
            return new ArrayList<>(detections);
        }
    }

    public void suspendDetection() { detect = false; }
    public void resumeDetection()  { detect = true; }
    public void suspend()          { suspend = true; }
    public void resume()           { suspend = false; }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Nothing per-resolution to do; area gates are per-track.
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        if (suspend) {
            return input; // return unmodified
        }
        lastCaptureNanos = captureTimeNanos;
        List<BallBlob> out = new ArrayList<>();
        if (detect) findBlobs(input, out);
        for (BallBlob b : out) b.normalize(input.cols(), input.rows());

        synchronized (lock) { detections = out; }

        if (!detect) {
            return input; // paused: no debug overlay, and no wasted full-frame clone
        }

        // Optional overlay for debug (VisionPortal can display it)
        Mat output = input.clone();
        drawOverlay(output, out);
        return output;
    }

    @Override
    public void onDrawFrame(android.graphics.Canvas canvas, int onscreenWidth, int onscreenHeight,
                            float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // No custom canvas drawing needed.
    }

    /** Morph + contour extraction + shape/size gating, shared by both tracks.
     *  A subclass builds a per-color mask with its own threshold math, then
     *  calls this. Ownership of `mask` transfers here (it is released). */
    protected void blobsFromMask(Mat mask, BallColor color,
                                 double minAreaPx, double maxAreaPx,
                                 double minAspect, double maxAspect,
                                 double minFill, double maxFill,
                                 List<BallBlob> out) {
        if (USE_MORPH) {
            Mat el = Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,
                                                   new org.opencv.core.Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, el);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, el);
            el.release();
        }
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hier = new Mat();
        Imgproc.findContours(mask, contours, hier, Imgproc.RETR_EXTERNAL,
                             Imgproc.CHAIN_APPROX_SIMPLE);
        hier.release();
        mask.release();
        for (MatOfPoint ct : contours) {
            double area = Imgproc.contourArea(ct);
            Rect r = Imgproc.boundingRect(ct);
            ct.release();
            if (area < minAreaPx || area > maxAreaPx) continue;
            if (r.width <= 0 || r.height <= 0) continue;
            double aspect = (double) r.width / r.height;
            if (aspect < minAspect || aspect > maxAspect) continue;
            double fill = area / (double) (r.width * r.height);
            if (fill < minFill || fill > maxFill) continue;
            out.add(new BallBlob(color, r, area));
        }
    }

    private void drawOverlay(Mat frame, List<BallBlob> blobs) {
        for (BallBlob b : blobs) {
            Scalar col = b.color == BallColor.YELLOW ? new Scalar(0, 255, 255)
                       : b.color == BallColor.RED    ? new Scalar(0, 0, 255)
                       : new Scalar(255, 0, 0);
            Imgproc.rectangle(frame, b.rect, col, 2);
            Imgproc.putText(frame, b.color.name(),
                            new Point(b.rect.x, b.rect.y - 6),
                            Imgproc.FONT_HERSHEY_PLAIN, 1.0, col, 2);
        }
    }
}