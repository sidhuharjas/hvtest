package org.hivevisionftc.vision;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class LimelightSource implements DetectionSource {
    private final Limelight3A limelight;

    public LimelightSource(Limelight3A limelight, int pipeline, int pollRateHz) {
        this.limelight = limelight;
        this.limelight.pipelineSwitch(pipeline);
        this.limelight.setPollRateHz(pollRateHz);
        this.limelight.start();
    }

    @NonNull
    @Override
    public List<Detection> latest() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return Collections.emptyList();
        }

        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null) {
            return Collections.emptyList();
        }

        List<Detection> out = new ArrayList<>(detections.size());
        for (LLResultTypes.DetectorResult detection : detections) {
            out.add(new Detection(
                    detection.getClassId(),
                    detection.getTargetXDegrees(),
                    detection.getTargetYDegrees(),
                    detection.getConfidence(),
                    detection.getTargetArea()));
        }

        return out;
    }

    @Override
    public long stalenessMs() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return -1;
        }

        return result.getStaleness();
    }
}
