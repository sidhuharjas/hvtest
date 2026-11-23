package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class Detection {
    private Pose pose;
    private double tx;
    private double ty;
    private double confidence;
    private double area;
    private int classId;
    private String className;

    public Detection(Pose robotPose, VisionConfig visionConfig, double tx, double ty, double confidence, double area) {
        this.tx = tx;
        this.ty = ty;
        this.confidence = confidence;
        this.area = area;
        // A detection built by hand has no class; -1 is UNKNOWN. Leaving this
        // at 0 made Classifier read every hand-built detection as pollen.
        this.classId = -1;
        pose = findDetectionPose(robotPose, visionConfig, tx, ty);
        // TODO: class id and class name. VisionPortal probably reports something
        // different (enums?), so this needs another pass.
    }

    public Detection(Pose robotPose, VisionConfig visionConfig, LLResultTypes.DetectorResult detectorResult) {
        tx = detectorResult.getTargetXDegrees();
        ty = detectorResult.getTargetYDegrees();
        confidence = detectorResult.getConfidence();
        area = detectorResult.getTargetArea();
        pose = findDetectionPose(robotPose, visionConfig, detectorResult);
        classId = detectorResult.getClassId();
        className = detectorResult.getClassName();
    }



    public Pose getPose() {
        return pose;
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public double getTx() {
        return tx;
    }

    public void setTx(double tx) {
        this.tx = tx;
    }

    public double getTy() {
        return ty;
    }

    public void setTy(double ty) {
        this.ty = ty;
    }

    public double getConfidence() {
        return confidence;
    }

    public void setConfidence(double confidence) {
        this.confidence = confidence;
    }

    public double getArea() {
        return area;
    }

    public void setArea(double area) {
        this.area = area;
    }

    public int getClassId() {
        return classId;
    }

    public void setClassId(int classId) {
        this.classId = classId;
    }

    public String getClassName() {
        return className;
    }

    public void setClassName(String className) {
        this.className = className;
    }

    // Both constructors call findDetectionPose(...), but the method was missing
    // from the file, so the module did not compile. The projection follows
    // VisionConfig's conventions: pitch up is positive, offsets are from the
    // robot center.

    private static Pose findDetectionPose(Pose robotPose, VisionConfig visionConfig,
                                          double tx, double ty) {
        return project(robotPose, visionConfig, tx, ty);
    }

    private static Pose findDetectionPose(Pose robotPose, VisionConfig visionConfig,
                                          LLResultTypes.DetectorResult detectorResult) {
        return project(robotPose, visionConfig,
                detectorResult.getTargetXDegrees(), detectorResult.getTargetYDegrees());
    }

    private static Pose project(Pose robotPose, VisionConfig cfg, double txDeg, double tyDeg) {
        if (robotPose == null || cfg == null) return new Pose(0, 0, 0);
        // Ray elevation above the horizon: camera pitch plus the target's ty.
        double elevationDeg = cfg.getCameraPitchDeg() + tyDeg;
        // Ground range to a floor target; unusable when the ray is level or up.
        double range = elevationDeg < -0.5
                ? cfg.getCameraHeightIn() / Math.tan(Math.toRadians(-elevationDeg))
                : 0;
        double tx = Math.toRadians(txDeg);
        // Robot frame: +x forward, +y left (tx is positive to the right).
        double fwd = range * Math.cos(tx) + cfg.getCameraForwardIn();
        double left = -range * Math.sin(tx) + cfg.getCameraLeftIn();
        double h = robotPose.getHeading();
        double x = robotPose.getX() + fwd * Math.cos(h) - left * Math.sin(h);
        double y = robotPose.getY() + fwd * Math.sin(h) + left * Math.cos(h);
        return new Pose(x, y, h);
    }
}
