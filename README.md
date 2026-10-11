# Hive Ball Chase

Limelight 3A ball detection, target locking and chase drivers for FTC robots: a perception core
(`BallTracker`), a no-odometry chase controller, an obstacle-avoiding Pedro Pathing + Marrow Weaver
hybrid chaser (`BallChaseWeaver`), a one-line `BallHunt` and a fluent verb API (`BallWrangler`).
Built to pick up BioBuzz game elements (`yellow_pollen` / `red_nectar` / `blue_nectar`) with a
Limelight 3A SSD neural detector.

Part of [Hive Vision](https://github.com/hivevisionftc/Hive-Vision) (model, training and calibration tooling).

## Installation

Pick the artifact that matches the Pedro Pathing version in your robot project. **Add exactly one.**

| Artifact | FTC SDK | Pedro Pathing | Status |
|----------|---------|---------------|--------|
| `org.hivevisionftc:ball-chase-weaver` | 12.x | 3.x | **Current.** Marrow Weaver paths that route around obstacles; `ChaseDriver`/`PickupHandler` seams so it is not locked to one drivetrain or intake. |
| `org.hivevisionftc:ball-chase` | 12.x (verified 12.0.0) | 3.x (verified 3.0.1) | **Archived.** `BallChaseFollower` is `@Deprecated`; kept building so mid-season teams are not broken. No new features. |
| `org.hivevisionftc:ball-chase-legacy` | 11.x (verified 11.2.1) | 2.x (verified 2.1.2) | **Archived.** Has `HiveConfig` and `PickupConfirmer`. Removal candidate. |

> **Where BallChaseFollower went.** Straight-line travel can't plan around obstacles,
> and the hybrid driver had no seam for another drivetrain, so development moved to
> `ball-chase-weaver`. It keeps the same perception core and the same state machine,
> replaces the TRAVEL leg with one cost-ordered [Marrow Weaver](https://github.com/Skeleton-Army/Marrow)
> route through every remembered ball that bends around obstacle zones, and adds a
> `ChaseDriver` adapter interface and a `PickupHandler`. `BallHunt` and every tracker
> verb are unchanged.

```gradle
// TeamCode/build.gradle
dependencies {
    implementation 'org.hivevisionftc:ball-chase-weaver:1.0.0'
}
```

The artifacts publish to Maven Central, which the FTC Gradle setup already includes. The library only
compiles against the FTC SDK and Pedro Pathing (`compileOnly`), so your robot project supplies them;
`ball-chase-weaver` also pulls in Marrow (Weaver path generation) for you.

Verified against a stock
[FtcRobotController](https://github.com/FIRST-Tech-Challenge/FtcRobotController) checkout
(FTC SDK 12.0.0, AGP 8.13.2, Gradle 9.1.0): no repository edits, and copy-paste TeamCode OpModes
live in [`docs/examples/`](docs/examples/). Full setup:
[ball-chase-weaver/README.md](ball-chase-weaver/README.md).

If you use other versions, compatibility is not guaranteed.

> **Package change.** Classes used to live in `org.firstinspires.ftc.teamcode`. They are now in
> `org.hivevisionftc.ballchase` (`...ballchase.wrapper` for the verb API). Update your imports.

## Beta testing (before the artifact is on Central)

The artifact is not on Maven Central yet, so testers install it from source. Send this section to
whoever is testing — it is self-contained.

**1. Publish the library locally** (needs JDK 17+ and the Android SDK, the same setup that builds
FtcRobotController):

```bash
git clone https://github.com/sidhuharjas/hvtest.git
cd hvtest
./gradlew :ball-chase-weaver:deployLocal   # -> org.hivevisionftc:ball-chase-weaver:1.0.0 in ~/.m2
```

**2. Two edits in your FtcRobotController project.** `build.dependencies.gradle`:

```gradle
repositories {
    mavenLocal()
    mavenCentral()
    google()
}
```

`TeamCode/build.gradle`:

```gradle
dependencies {
    implementation project(':FtcRobotController')   // already there
    implementation 'com.pedropathing:core:3.0.1'
    implementation 'com.pedropathing:revhub:3.0.1'  // REV Hub drivetrain/localizer
    implementation 'org.hivevisionftc:ball-chase-weaver:1.0.0'
}
```

**3. OpModes.** Copy [`docs/examples/`](docs/examples/) into TeamCode and tune
`SampleFollowerFactory` for your robot (motor/encoder names, pod offsets, ticks-to-inches, hub
orientation). Hardware names are `limelight` and `intake` — edit the OpModes if yours differ.

**4. Camera.** Upload `best_limelight3a_ssd_mobilenetv2_300x300.tflite` + `labels.txt` (Hive Vision
repo, `neural-net/weights/`) at `http://limelight.local:5801`: Pipeline Type **Neural Detector**,
runtime engine **CPU**, input **300 x 300**, threshold **0.35 - 0.45**. Confirm colored boxes
appear on the stream before running anything.

**5. Run.** `AutoBallHuntOpMode` first (the one-liner), then `ManualBallChaseOpMode` (the full
loop) — add an obstacle with `hunt.addObstacle(new CircleZone(new Point(x, y), r))` to test
avoidance.

**Report back:**

- a ✅/❌ for each state: `SCAN` → `TRAVEL` → `TURN` → `CHASE` → `PICKUP` (telemetry shows the state)
- video of the run, especially whether the path visibly bends around the obstacle
- every exception from the Driver Station, verbatim
- field size, obstacle coordinates, and any tunables you had to change (the `BallChaseWeaver.*` statics)

## Usage

### Teleop (no odometry)

```java
BallTracker tracker = new BallTracker(limelight);
tracker.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);   // or CLASSES_ALL

BallChaseController chase = new BallChaseController(tracker, lf, rf, lb, rb, intake);
chase.setMaxPickups(3);
chase.start();
while (opModeIsActive() && !chase.isDone()) {
    chase.update();
    chase.addTelemetry(telemetry);
}
chase.abort();
```

### Auto with Pedro

```java
// YOU own follower.update() - this class never calls it
BallChaseWeaver hunt = new BallChaseWeaver(follower, tracker, intake);
hunt.setAllowedClasses(BallTracker.CLASSES_RED_BLUE);
hunt.addObstacle(new CircleZone(new Point(72, 72), 18));   // optional: route around it
hunt.setMaxPickups(3); hunt.setTimeBudgetSec(20);
hunt.start();
while (opModeIsActive() && !hunt.isDone()) {
    follower.update();
    hunt.update();
    hunt.addTelemetry(telemetry);
}
hunt.abort();
```

On the archived `ball-chase` artifact the same loop uses `BallChaseFollower`
(straight-line travel, no obstacles).

### Auto, the one-liner

```java
int picked = new BallHunt(follower, limelight, intake)
        .reds().collect(2).within(20)
        .go(this);          // owns follower.update(), streams telemetry, aborts cleanly
```

Fluent options: `.reds()` `.blues()` `.yellows()` `.alliance()` (default) `.everything()` ·
`.collect(n)` (≤0 = unlimited) · `.within(sec)` · `.avoid(zone)`.

Class-level reference (state machines, lock semantics, the verb API): [docs/MODULES.md](docs/MODULES.md).

## Calibrate before running

On `ball-chase-legacy` every tunable is a `public static` field in `HiveConfig`, read at point of use, so
you can edit it from your own code or a config system (e.g. Bylazar Configurables) before starting a hunt.
On `ball-chase-weaver` and `ball-chase` the same constants are fields on `BallTracker`,
`BallChaseController`, `BallChaseWeaver` and `BallChaseFollower`. The measurement procedure and coordinate
sign conventions are in [docs/calibration.md](docs/calibration.md).

| Constant | Meaning |
|----------|---------|
| `CAM_PITCH_DEG`, `CAM_H`, `BALL_H` | Camera tilt below horizontal, lens height, ball-center height (inches). |
| `CAM_X_OFFSET`, `CAM_Y_OFFSET` | Camera mount offset from robot center (forward +X, LEFT +Y, inches). |
| `MIN_CONF`, `MAX_STALENESS_MS` | Detection gating. Staleness is in milliseconds; the 3A reports confidence 0-1. |
| Class ids 0/1/2 | Must match your Limelight pipeline's label list. Nothing checks the model at runtime. |
| `HFOV_DEG` / `VFOV_DEG` | Camera field of view, used for field projection. |
| Drive/turn/coast gains, search/pickup timing | Everything the drivers read at point of use. |

## Project layout

```
ball-chase-weaver/    FTC 12 + Pedro 3 + Marrow Weaver artifact   (current)
ball-chase/           FTC 12 + Pedro 3 artifact                    (archived)
ball-chase-legacy/    FTC 11 + Pedro 2 artifact                    (archived)
docs/                 MODULES.md (class reference), calibration.md
```

Each artifact is `src/main` = library, `src/test` = desktop suites (no robot, no camera).

The robot-side OpModes from the original repo (`RobotTestBench`, `BallChaseControllerTest`,
`TeleOpWorking`, `BallChaseOpMode`, the `older/` demos) are not part of the library. They remain in the
[Hive-Vision repository](https://github.com/hivevisionftc/Hive-Vision/tree/main/ftc_ball_chase_lib).

## Building and testing

```bash
./gradlew test               # desktop self-test suites (no robot, no camera)
./gradlew assembleRelease    # AARs plus sources/javadoc jars
./gradlew spotlessCheck      # formatting check (spotlessApply to format)
```

Needs a JDK 17+ and the Android SDK (`ANDROID_HOME` or `local.properties` with `sdk.dir`).

## Publishing to Maven Central

Publishing uses the [deployer](https://github.com/deepmedia/Deployer) plugin and the Central Portal,
configured in the root `build.gradle.kts`.

One-time setup:

1. Verify your namespace on the [Central Portal](https://central.sonatype.com/)
   (`org.hivevisionftc` is the Maven group in the root `build.gradle.kts` — verify the matching
   domain, or change `group` if you verify a different one).
2. Fill in the developer/repository details at the top of `build.gradle.kts` (`DeployerExtension`).
3. Create a GPG key and publish its public key to a keyserver.

Provide the credentials as environment variables (or Gradle properties):

| Secret | Meaning |
|--------|---------|
| `SIGNING_KEY` | Armored private GPG key |
| `SIGNING_PASSWORD` | Passphrase of that key |
| `CENTRAL_PORTAL_USER` / `CENTRAL_PORTAL_PASSWORD` | Central Portal user token pair |

| Task | Does |
|------|------|
| `./gradlew deployLocal` | Publish all modules to Maven Local (try this first) |
| `./gradlew deployNexusSnapshot` | Publish snapshots (version must end in `-SNAPSHOT`) |
| `./gradlew deployCentralPortal` | Publish a release to Maven Central |

Bump `libVersion` in `build.gradle.kts` before each release.

## License

MIT, see [LICENSE](LICENSE).
