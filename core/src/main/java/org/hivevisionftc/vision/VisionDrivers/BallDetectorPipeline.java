/*
 * Hive Vision — Control Hub track (OpenCV, no model)
 * Color-threshold ball detector for FTC. Runs on the Control Hub via
 * VisionPortal (VisionProcessor), no coprocessor required. The shared
 * candidate API, threading, and contour gating live in BallBlobPipeline;
 * this class supplies only the HSV color math.
 *
 * HSV ranges and geometry gates were learned from the Limelight track's
 * YOLO model on real match footage (fit_hsv_from_yolo.py).
 * Re-tune on your actual field camera with hsv_tuner.py and copy the
 * constants here if lighting differs.
 *
 * Metrics (vs YOLO truth on the development match video):
 *   yellow: recall 68.7%, precision 39.9%
 *   red   : recall 80.1%, precision 26.0%   (red panels/tape are FP-heavy)
 *   blue  : recall 82.3%, precision 65.5%
 *
 * Typical HSV (OpenCV H 0..180, S/V 0..255):
 *   yellow: H 9..33,   S 90..255,  V 45..238
 *   red   : H 0..15 and 170..180,  S 81..255,  V 45..232
 *   blue  : H 102..128, S 80..255, V 45..247
 */
package org.hivevisionftc.vision.VisionDrivers;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BallDetectorPipeline extends BallBlobPipeline {

    /* ------------------------------------------------------------------
     * 1) HSV segments (lower/upper per color). Red is matched with two
     *    segments because its hue wraps through 0.
     * ----------------------------------------------------------------*/
    private static final Scalar YELLOW_LO = new Scalar(9, 90, 45);
    private static final Scalar YELLOW_HI = new Scalar(33, 255, 238);

    private static final Scalar RED_LO_A = new Scalar(0, 81, 45);
    private static final Scalar RED_HI_A = new Scalar(15, 255, 232);
    private static final Scalar RED_LO_B = new Scalar(170, 81, 45);
    private static final Scalar RED_HI_B = new Scalar(180, 255, 232);

    private static final Scalar BLUE_LO = new Scalar(102, 80, 45);
    private static final Scalar BLUE_HI = new Scalar(128, 255, 247);

    /* ------------------------------------------------------------------
     * 2) Shape/size gating (pixels at camera resolution).
     *    Real balls on the FTC field are small-to-mid and roughly square;
     *    robot side panels are giant or elongated. Tune for your camera.
     * ----------------------------------------------------------------*/
    private static final int   MIN_AREA_PX  = 900;    // kills speckle patches
    private static final int   MAX_AREA_PX  = 12000;  // kills big robot panels
    private static final double MIN_ASPECT  = 0.71;   // near-square balls only
    private static final double MAX_ASPECT  = 1.4;
    private static final double MIN_FILL    = 0.5;    // contourArea / rectArea
    private static final double MAX_FILL    = 1.3;

    /** Typical real-ball contour area per color (px at camera resolution). */
    private static final double EXPECT_YELLOW_AREA = 3500;
    private static final double EXPECT_RED_AREA    = 4200;
    private static final double EXPECT_BLUE_AREA   = 4200;

    @Override
    protected double expectArea(BallColor c) {
        switch (c) {
            case YELLOW: return EXPECT_YELLOW_AREA;
            case BLUE:   return EXPECT_BLUE_AREA;
            default:     return EXPECT_RED_AREA;
        }
    }

    @Override
    protected void findBlobs(Mat input, List<BallBlob> out) {
        maskAndFind(input, BallColor.YELLOW, YELLOW_LO, YELLOW_HI, null, null, out);
        maskAndFind(input, BallColor.RED,    RED_LO_A, RED_HI_A, RED_LO_B, RED_HI_B, out);
        maskAndFind(input, BallColor.BLUE,   BLUE_LO, BLUE_HI, null, null, out);
    }

    private void maskAndFind(Mat input, BallColor color, Scalar loA, Scalar hiA,
                             Scalar loB, Scalar hiB, List<BallBlob> out) {
        Mat hsv = new Mat();
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);
        Mat mask = new Mat();
        if (loB != null) { // red wrap: union of two masks
            Mat m2 = new Mat();
            Core.inRange(hsv, loA, hiA, mask);
            Core.inRange(hsv, loB, hiB, m2);
            Core.bitwise_or(mask, m2, mask);
            m2.release();
        } else {
            Core.inRange(hsv, loA, hiA, mask);
        }
        hsv.release();
        blobsFromMask(mask, color, MIN_AREA_PX, MAX_AREA_PX,
                      MIN_ASPECT, MAX_ASPECT, MIN_FILL, MAX_FILL, out);
    }
}