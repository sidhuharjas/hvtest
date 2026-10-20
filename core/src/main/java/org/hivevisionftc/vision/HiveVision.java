package org.hivevisionftc.vision;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Function;

public class HiveVision {
    private final DetectionSource source;
    private final VisionConfig config;
    private final Set<BiobuzzClasses> allowedClasses;

    public HiveVision(DetectionSource source, VisionConfig config) {
        this.source = source;
        this.config = config;
        this.allowedClasses = new HashSet<>(Arrays.asList(BiobuzzClasses.values()));
    }

    public HiveVision(Limelight3A limelight, VisionConfig config) {
        this(new LimelightSource(limelight, config.getPipeline(), config.getPollingRateHz()), config);
    }

    public List<Detection> getRawDetections() {
        return source.latest();
    }

    public long getStalenessMs() {
        return source.stalenessMs();
    }

    public HiveVision setAllowedClasses(BiobuzzClasses... classes) {
        allowedClasses.clear();
        allowedClasses.addAll(Arrays.asList(classes));
        return this;
    }

    public HiveVision addAllowedClass(BiobuzzClasses clazz) {
        allowedClasses.add(clazz);
        return this;
    }

    public boolean isAllowed(BiobuzzClasses clazz) {
        return allowedClasses.contains(clazz);
    }

    public Set<BiobuzzClasses> getAllowed() {
        return allowedClasses;
    }

    /** Creates an empty {@link DetectionList} bound to this vision instance; call {@code fetch(...)} to fill it. */
    public DetectionList detections() {
        return new DetectionList();
    }

    public class DetectionList {
        private final List<Detection> detectionList;
        private FieldPose robotPose;

        private DetectionList() {
            detectionList = new ArrayList<>();
        }

        // ─── Extension point ─────────────────────────────────────────────────────

        /**
         * Hands this list to {@code fn} and returns whatever it produces. This is how optional modules
         * (e.g. the Weaver integration) add behaviour without the core depending on them:
         * <pre>{@code
         * PathResult path = vision.detections().fetch(pose).sortByDistance().with(WeaverRoute.from(pose));
         * }</pre>
         */
        public <R> R with(Function<? super DetectionList, ? extends R> fn) {
            return fn.apply(this);
        }

        /** The robot pose passed to the last {@code fetch(...)}, or {@code null} if never fetched. */
        public FieldPose getRobotPose() {
            return robotPose;
        }

        public DetectionList fetch(double robotX, double robotY) {
            // Clear list to avoid stale data
            detectionList.clear();

            long staleness = getStalenessMs();
            if (staleness < 0 || staleness > config.getMaxStalenessMs()) {
                return this; // Return empty list if new data is too old
            }

            List<Detection> detections = source.latest();
            robotPose = new FieldPose(robotX, robotY);

            for (Detection detection : detections) {
                if (!allowedClasses.contains(detection.getBallClass())) continue; // Skip disallowed colors
                if (!config.ignoreConfidence() && detection.getConfidence() < config.getMinConfidence())
                    continue; // Skip low confidence detections

                detectionList.add(getAbsolutePosition(detection, robotPose));
            }

            return this;
        }

        public DetectionList fetch(FieldPose robotPose) {
            return fetch(robotPose.getX(), robotPose.getY());
        }
        // ─── Sorting ─────────────────────────────────────────────────────────────

        /** Sorts by straight-line distance from the robot's pose (closest first). */
        public DetectionList sortByDistance() {
            detectionList.sort(Comparator.comparingDouble(
                    a -> Math.hypot(a.getX() - robotPose.getX(), a.getY() - robotPose.getY())));

            return this;
        }

        /** Sorts by color ordinal ascending (POLLEN, NECTAR_RED, NECTAR_BLUE, UNKNOWN). */
        public DetectionList sortByColor() {
            detectionList.sort(Comparator.comparingInt(i -> i.getBallClass().getId()));
            return this;
        }

        /** Sorts by area descending (largest first). */
        public DetectionList sortBySize() {
            detectionList.sort(Comparator.comparingDouble(Detection::getArea).reversed());
            return this;
        }

        public DetectionList sortByConfidence() {
            detectionList.sort(
                    Comparator.comparingDouble(Detection::getConfidence).reversed());
            return this;
        }

        // ─── Filtering ───────────────────────────────────────────────────────────
        public DetectionList clampYDetections(double minY, double maxY) {
            detectionList.removeIf(d -> d.getY() < minY);
            detectionList.removeIf(d -> d.getY() > maxY);
            return this;
        }

        // ─── Terminals ───────────────────────────────────────────────────────────

        /** Returns the closest artifact, or {@code null} if the list is empty. */
        public Detection getClosest() {
            if (detectionList.isEmpty()) return null;
            return sortByDistance().detectionList.get(0);
        }

        public Detection getBiggest() {
            if (detectionList.isEmpty()) return null;
            return sortBySize().detectionList.get(0);
        }

        /** Returns a copy of the current artifact list. */
        public List<Detection> getDetections() {
            return new ArrayList<>(detectionList);
        }

        public int count() {
            return detectionList.size();
        }

        public boolean didDetect() {
            return !detectionList.isEmpty();
        }

        private double getDistance(double ty) {
            double angleGoalRag = Math.toRadians(config.getCamPitchDeg() + ty);
            return (config.getBallHeight() - config.getCamHeight()) / Math.tan(angleGoalRag);
        }

        private FieldPose getRelativePose(double distance, double tx) {
            double theta = Math.toRadians(tx);

            double deltaX = distance * Math.cos(theta) + config.getCamXOffset();
            double deltaY = -distance * Math.sin(theta) + config.getCamYOffset();

            return new FieldPose(deltaX, deltaY);
        }

        private FieldPose getAbsolutePosition(FieldPose relativePose, FieldPose robotPose) {
            double theta = robotPose.getHeading();
            double x = relativePose.getX() * Math.cos(theta) - relativePose.getY() * Math.sin(theta);
            double y = relativePose.getX() * Math.sin(theta) + relativePose.getY() * Math.cos(theta);

            return new FieldPose(robotPose.getX() + x, robotPose.getY() + y, 0);
        }

        private Detection getAbsolutePosition(Detection detection, FieldPose robotPose) {
            double distance = getDistance(detection.getY());
            FieldPose pose = getAbsolutePosition(getRelativePose(distance, detection.getX()), robotPose);
            return new Detection(detection, pose);
        }
    }
}
