# HiveVision Weaver

Optional module: plans a [Marrow Weaver](https://marrow.skeletonarmyftc.com/automations/weaver) route
through the detections the core library returns.

```gradle
implementation 'org.hivevisionftc:weaver:1.0.0'   // pulls in :core and Marrow
```


Example usage:
```java
PathResult path = WeaverRoute.from(pose)
        .avoid(new CircleZone(new Point(72, 72), 18))   // optional obstacles
        .maxTargets(4)                                   // optional cap
        .apply(vision.fetchLimelightDetections(pose));

if (path != null) follower.follow(PedroPathingConverter.toPath(path));
```

- `apply(...)` takes the `List<Detection>` from `fetchLimelightDetections(...)` or `fetchWebcamDetections(...)`;
  core never depends on Marrow.
- Returns `null` when nothing was detected.
- Detections must be in field coordinates, which is what the fetch methods produce.
