package org.hivevisionftc.vision.weaver;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import androidx.annotation.NonNull;

import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PathRoute;
import com.skeletonarmy.marrow.weaver.Weaver;
import com.skeletonarmy.marrow.weaver.WeaverGenerator;
import com.skeletonarmy.marrow.zones.CircleZone;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.Zone;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import org.hivevisionftc.vision.Detection;
import org.hivevisionftc.vision.DetectionSource;
import org.hivevisionftc.vision.FieldPose;
import org.hivevisionftc.vision.HiveVision;
import org.hivevisionftc.vision.VisionConfig;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Desktop self-test for the Weaver integration: no robot, no camera, no Android.
 *
 * <p>Routes are planned by the real Marrow Weaver. Most tests hand {@link WeaverRoute#plan} detections that are
 * already in field coordinates, so they pin down exactly what the integration feeds Weaver and what comes back,
 * independent of the camera projection in core. A couple of tests go through the real
 * {@code DetectionList.with(...)} chain.
 */
class WeaverRouteTest {
    private static final FieldPose START = new FieldPose(24, 24, 0.0);

    /** An obstacle sitting 3" off the straight line from START to (35, 24). */
    private static final Point OBSTACLE_CENTRE = new Point(30, 27);

    private static final double OBSTACLE_RADIUS = 4.0;

    @BeforeEach
    void smallRobot() {
        // Shrink the footprint so short test routes are not "blocked" by the default 18" robot.
        Weaver.resetToDefaults();
        Weaver.getConfig().robotWidth(1).robotHeight(1).clearance(0.5);
    }

    @AfterEach
    void restore() {
        Weaver.resetToDefaults();
    }

    /* ---------------- nothing to plan ---------------- */

    @Test
    void emptyListYieldsNoRoute() {
        HiveVision vision = visionSeeing();
        assertNull(vision.detections().fetch(START).with(WeaverRoute.from(START)));
    }

    @Test
    void planWithNoDetectionsYieldsNoRoute() {
        assertNull(WeaverRoute.from(START).plan(Collections.emptyList(), START));
    }

    @Test
    void planWithoutAStartPoseSaysSo() {
        assertThrows(
                IllegalStateException.class,
                () -> WeaverRoute.fromFetchPose().plan(Collections.singletonList(ballAt(60, 24)), null));
    }

    @Test
    void startPoseIsRequiredByFrom() {
        assertThrows(IllegalArgumentException.class, () -> WeaverRoute.from(null));
    }

    /* ---------------- what Weaver is fed ---------------- */

    @Test
    void aSingleDetectionBecomesTheRoutesOnlyTarget() {
        PathResult plan = WeaverRoute.from(START).plan(balls(84, 24), START);

        assertNotNull(plan);
        assertEquals(1, plan.getVisitedTargets().size());
        assertEquals(84.0, plan.getVisitedTargets().get(0).getX(), 0.5, "target keeps its field X");
        assertEquals(24.0, plan.getVisitedTargets().get(0).getY(), 0.5, "target keeps its field Y");
        assertTrue(plan.getUnvisitedTargets().isEmpty());
        assertFalse(plan.getSegments().isEmpty(), "an obstacle-free hop still produces a curve");
    }

    @Test
    void toPointsKeepsOrderAndCoordinates() {
        List<Point> points = WeaverRoute.toPoints(balls(10, 20, 30, 40, 50, 60));

        assertEquals(3, points.size());
        assertEquals(10.0, points.get(0).getX(), 1e-9);
        assertEquals(20.0, points.get(0).getY(), 1e-9);
        assertEquals(30.0, points.get(1).getX(), 1e-9);
        assertEquals(40.0, points.get(1).getY(), 1e-9);
        assertEquals(50.0, points.get(2).getX(), 1e-9);
        assertEquals(60.0, points.get(2).getY(), 1e-9);
    }

    @Test
    void aFieldFrameDetectionKeepsBothCoordinates() {
        // Regression: Detection(Detection, Pose) used to copy X into Y.
        Detection d = ballAt(3, 7);
        assertEquals(3.0, d.getX(), 1e-9);
        assertEquals(7.0, d.getY(), 1e-9);
    }

    /* ---------------- route ordering and caps ---------------- */

    @Test
    void routeVisitsTheTurnCheaperTargetFirstEvenWhenAnotherIsNearer() {
        // (18, 24) is only 6" behind the start; (60, 24) is 36" dead ahead. Weaver weighs distance against
        // turning, so the ahead target is visited first and the behind one closes the route - even though we
        // handed them over nearest-first.
        PathResult plan = WeaverRoute.from(START).plan(balls(18, 24, 60, 24), START);

        assertNotNull(plan);
        List<Point> visited = plan.getVisitedTargets();
        assertEquals(2, visited.size(), "both targets join one route");
        assertEquals(60.0, visited.get(0).getX(), 0.5, "the turn-cheaper ahead target goes first");
        assertEquals(18.0, visited.get(1).getX(), 0.5, "the behind target closes the route");
        assertTrue(plan.getUnvisitedTargets().isEmpty());
    }

    @Test
    void maxTargetsCapsTheRouteAndReportsTheRest() {
        PathResult plan = WeaverRoute.from(START).maxTargets(2).plan(balls(60, 24, 24, 60, 60, 60), START);

        assertNotNull(plan);
        assertEquals(2, plan.getVisitedTargets().size(), "the cap keeps the third target off this route");
        assertEquals(1, plan.getUnvisitedTargets().size(), "and reports it as unvisited");
    }

    @Test
    void noCapVisitsEverything() {
        PathResult plan = WeaverRoute.from(START).plan(balls(60, 24, 24, 60, 60, 60), START);

        assertNotNull(plan);
        assertEquals(3, plan.getVisitedTargets().size());
        assertTrue(plan.getUnvisitedTargets().isEmpty());
    }

    @Test
    void aNonPositiveCapMeansNoCap() {
        PathResult plan = WeaverRoute.from(START).maxTargets(-5).plan(balls(60, 24, 24, 60, 60, 60), START);

        assertNotNull(plan);
        assertEquals(3, plan.getVisitedTargets().size());
    }

    /* ---------------- obstacles ---------------- */

    @Test
    void routeBendsAroundAnObstacleOnTheStraightLine() {
        Zone obstacle = new CircleZone(OBSTACLE_CENTRE, OBSTACLE_RADIUS);

        PathResult plan = WeaverRoute.from(START).avoid(obstacle).plan(balls(35, 24), START);

        assertNotNull(plan);
        PathRoute route = plan.getPath();
        assertTrue(
                WeaverGenerator.isPathClear(route, Collections.singletonList(obstacle), Weaver.getConfig()),
                "the route keeps the configured clearance from the obstacle");
        double closest = closestApproach(plan, OBSTACLE_CENTRE);
        assertTrue(closest > OBSTACLE_RADIUS, "the route bends instead of clipping the obstacle, closest=" + closest);
    }

    @Test
    void avoidAcceptsAnyCollectionOfZones() {
        List<Zone> zones = new ArrayList<>();
        zones.add(new CircleZone(OBSTACLE_CENTRE, OBSTACLE_RADIUS));

        PathResult plan = WeaverRoute.from(START).avoid(zones).plan(balls(35, 24), START);

        assertTrue(closestApproach(plan, OBSTACLE_CENTRE) > OBSTACLE_RADIUS);
    }

    @Test
    void aTargetInsideAnObstacleIsDroppedAndReported() {
        // (35, 24) sits inside the zone, (24, 60) is clear. excludeBlockedTargets is on by default, so the blocked
        // one must come back as unvisited rather than being driven into.
        Zone blocker = new CircleZone(new Point(35, 24), 3.0);

        PathResult plan = WeaverRoute.from(START).avoid(blocker).plan(balls(35, 24, 24, 60), START);

        assertNotNull(plan);
        assertEquals(1, plan.getVisitedTargets().size(), "only the reachable target is visited");
        assertEquals(60.0, plan.getVisitedTargets().get(0).getY(), 0.5);
        assertEquals(1, plan.getUnvisitedTargets().size(), "the blocked target is reported");
    }

    /* ---------------- through DetectionList.with ---------------- */

    @Test
    void withReturnsWhateverTheFunctionProduces() {
        HiveVision vision = visionSeeing();
        assertEquals(42, (int) vision.detections().with(list -> 42));
    }

    @Test
    void theChainPlansFromTheFetchPoseWhenNoStartIsGiven() {
        HiveVision vision = visionSeeing(new Detection(1, 0, 10, 0.9, 1));
        FieldPose robot = new FieldPose(72, 72, 0.0);

        PathResult plan = vision.detections().fetch(robot).with(WeaverRoute.fromFetchPose());

        assertNotNull(plan);
        assertEquals(1, plan.getVisitedTargets().size(), "the one detection became the one target");
        Point first = plan.getPath().getSegments().get(0).sample(5).get(0);
        assertEquals(72.0, first.getX(), 1.0, "the route starts at the pose the list was fetched with");
        assertEquals(72.0, first.getY(), 1.0);
    }

    @Test
    void anExplicitStartBeatsTheFetchPose() {
        HiveVision vision = visionSeeing(new Detection(1, 0, 10, 0.9, 1));

        PathResult plan = vision.detections()
                .fetch(new FieldPose(72, 72, 0.0))
                .with(WeaverRoute.from(new FieldPose(50, 72, 0.0)));

        assertNotNull(plan);
        Point first = plan.getPath().getSegments().get(0).sample(5).get(0);
        assertEquals(50.0, first.getX(), 1.0, "the explicit start wins");
    }

    @Test
    void orderedKeepsTheOrderTheDetectionsWereGivenIn() {
        // Without ordered(), (60,24) is visited first (see routeVisitsTheTurnCheaperTarget...).
        PathResult plan = WeaverRoute.from(START).ordered().plan(balls(18, 24, 60, 24), START);

        assertNotNull(plan);
        List<Point> visited = plan.getVisitedTargets();
        assertEquals(2, visited.size());
        assertEquals(18.0, visited.get(0).getX(), 0.5, "given order wins over cost order");
        assertEquals(60.0, visited.get(1).getX(), 0.5);
    }

    /* ---------------- harness ---------------- */

    /** A detection already in field coordinates, the way {@code fetch(...)} leaves them. */
    private static Detection ballAt(double x, double y) {
        return new Detection(new Detection(1, 0, 0, 0.9, 1), new FieldPose(x, y));
    }

    /** Field-frame detections from a flat x,y,x,y... list. */
    private static List<Detection> balls(double... xy) {
        List<Detection> out = new ArrayList<>();
        for (int i = 0; i + 1 < xy.length; i += 2) out.add(ballAt(xy[i], xy[i + 1]));
        return out;
    }

    /** Distance from {@code centre} to the nearest sample on the plan's first curve. */
    private static double closestApproach(PathResult plan, Point centre) {
        double closest = Double.MAX_VALUE;
        for (Point p : plan.getPath().getSegments().get(0).sample(41)) {
            closest = Math.min(closest, Math.hypot(p.getX() - centre.getX(), p.getY() - centre.getY()));
        }
        return closest;
    }

    private static HiveVision visionSeeing(Detection... detections) {
        final List<Detection> list = Arrays.asList(detections);
        DetectionSource source = new DetectionSource() {
            @NonNull
            @Override
            public List<Detection> latest() {
                return list;
            }

            @Override
            public long stalenessMs() {
                return 0;
            }
        };
        return new HiveVision(source, new VisionConfig());
    }
}
