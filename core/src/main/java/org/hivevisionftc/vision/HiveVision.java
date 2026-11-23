package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.hivevisionftc.vision.VisionDrivers.LimelightDriver;
import org.hivevisionftc.vision.VisionDrivers.WebcamDriver;

import java.util.ArrayList;

public class HiveVision {
    Limelight3A limelight;
    VisionConfig visionConfig;
    LimelightDriver limelightDriver;
    WebcamDriver webcamDriver;

    public HiveVision(Limelight3A limelight, VisionConfig visionConfig) {
        this.limelight = limelight;
        this.visionConfig = visionConfig;
        limelightDriver = new LimelightDriver(limelight, visionConfig);
    }

    public ArrayList<Detection> fetchLimelightDetections(Pose robotPose) {
        return limelightDriver.fetchLatestDetections(robotPose);
    }

    // Webcam track. Mirrors the Limelight path so callers can pick a camera
    // without anything above the drivers changing shape.

    public HiveVision(WebcamName webcam, VisionConfig visionConfig,
                      double webcamHFovDeg, double webcamVFovDeg) {
        this.visionConfig = visionConfig;
        webcamDriver = new WebcamDriver(webcam, visionConfig, webcamHFovDeg, webcamVFovDeg);
    }

    public ArrayList<Detection> fetchWebcamDetections(Pose robotPose) {
        return webcamDriver.fetchLatestDetections(robotPose);
    }

    /** The webcam driver, for frame-age checks and shutdown. Null on the Limelight track. */
    public WebcamDriver webcamDriver() {
        return webcamDriver;
    }
}
