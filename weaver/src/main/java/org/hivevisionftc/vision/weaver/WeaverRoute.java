package org.hivevisionftc.vision.weaver;

import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverBuilder;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.function.Function;
import org.hivevisionftc.vision.Detection;
import org.hivevisionftc.vision.Pose;

/**
 * Plans a Marrow Weaver route through a list of detections.
 *
 * <pre>{@code
 * PathResult path = WeaverRoute.from(pose).maxTargets(4)
 *         .apply(vision.fetchLimelightDetections(pose));
 *
 * if (path != null) follower.follow(PedroPathingConverter.toPath(path));
 * }</pre>
 *
 * <p>Detections must already hold field-frame positions (that is what
 * {@code fetchLimelightDetections(...)} produces).
 *
 * <p>The core rewrite removed {@code HiveVision.DetectionList}, so {@code apply}
 * takes the {@code List<Detection>} the core returns, and {@code toPoint} reads
 * the pose through {@code getPose()}. The planning logic is unchanged.
 */
public final class WeaverRoute implements Function<List<Detection>, PathResult> {
    private Pose start;
    private Pose end;
    private final List<Zone> obstacles;
    private int maxTargets;
    private boolean ordered = false;

    private WeaverRoute(Pose start) {
        this.start = start;
        obstacles = new ArrayList<>();
    }

    /** Starts the route at {@code start} (field inches, heading radians). */
    public static WeaverRoute from(Pose start) {
        if (start == null) throw new IllegalArgumentException("start must not be null");
        return new WeaverRoute(start);
    }

    /** Adds obstacle zones (field inches) the route must bend around. */
    public WeaverRoute avoid(Zone... zones) {
        return avoid(Arrays.asList(zones));
    }

    /**
     * Adds obstacle zones the route must bend around.
     *
     * @param zone zone to avoid
     * @return this
     */
    public WeaverRoute addObstacle(Zone zone) {
        return avoid(zone);
    }

    /** Adds obstacle zones (field inches) the route must bend around. */
    public WeaverRoute avoid(Collection<? extends Zone> zones) {
        obstacles.addAll(zones);
        return this;
    }

    /** Visits at most {@code n} detections (Weaver keeps the best ones). {@code 0} or less means all of them. */
    public WeaverRoute maxTargets(int n) {
        maxTargets = n;
        return this;
    }

    /**
     * Set path targets as ordered
     *
     * @return this
     */
    public WeaverRoute ordered() {
        this.ordered = true;
        return this;
    }

    /**
     * Sets the destination pose for the path.
     *
     * @param end the destination pose
     * @return this
     */
    public WeaverRoute end(Pose end) {
        this.end = end;
        return this;
    }

    /**
     * Plans the route.
     *
     * @return the planned route, or {@code null} when the list has no detections to route to
     * @throws IllegalStateException if no start was given and the list was never fetched
     * @throws RuntimeException whatever Weaver throws for degenerate geometry; catch it if you want to fall back
     */
    @Override
    public PathResult apply(List<Detection> detections) {
        if (detections.isEmpty()) return null;
        return plan(detections, start);
    }

    /**
     * Plans a route through already field-frame detections.
     *
     * @param fieldDetections detections whose pose is in field inches
     * @param from the pose the route starts from
     * @return the planned route, or {@code null} when there are no detections
     * @throws IllegalStateException if {@code from} is {@code null}
     */
    public PathResult plan(List<Detection> fieldDetections, Pose from) {
        if (fieldDetections.isEmpty()) return null;
        if (from == null) {
            throw new IllegalStateException(
                    "No start pose: pass one to WeaverRoute.from(pose)");
        }

        WeaverBuilder builder = Weaver.builder();
        if (ordered) {
            builder.ordered();
        }

        if (end != null) {
            builder.end(new Point(end.getX(), end.getY(), end.getHeading()));
        }

        // Marrow treats maxTargets(0) as "no cap", so it can be passed through unconditionally.
        return builder.start(new Point(from.getX(), from.getY(), from.getHeading()))
                .targets(toPoints(fieldDetections))
                .maxTargets(maxTargets)
                .obstacles(obstacles)
                .generate();
    }

    /** Converts one field-frame detection to a Weaver target (heading is derived from the direction of travel). */
    public static Point toPoint(Detection detection) {
        Pose pose = detection.getPose();
        return new Point(pose.getX(), pose.getY());
    }

    /** Converts field-frame detections to Weaver targets, preserving order. */
    public static List<Point> toPoints(Collection<Detection> detections) {
        List<Point> points = new ArrayList<>(detections.size());
        for (Detection d : detections) points.add(toPoint(d));
        return points;
    }
}
