android {
    namespace = "org.hivevisionftc.vision.weaver"
    compileSdk = 35

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    testOptions {
        unitTests.isReturnDefaultValues = true
    }

    publishing {
        singleVariant("release") {
            withSourcesJar()
            withJavadocJar()
        }
    }
}

tasks.withType<Test>().configureEach {
    useJUnitPlatform()
}

dependencies {
    // Public API exposes HiveVision.DetectionList, so core is an api dependency.
    api(project(":core"))

    // Public API returns Marrow's PathResult and takes its Zone, so consumers need it on their compile classpath.
    // Non-transitive: only the weaver/zones packages are used, which keeps Marrow's JSON layer (Jackson)
    // off the robot.
    api("com.skeletonarmyftc.marrow:core:1.2.2") { isTransitive = false }

    // Supplied by the robot project at runtime. Core's HiveVision references Limelight3A.
    compileOnly("org.firstinspires.ftc:RobotCore:12.0.0")
    compileOnly("org.firstinspires.ftc:Hardware:12.0.0")
    implementation("androidx.annotation:annotation:1.6.0")

    testImplementation("org.firstinspires.ftc:RobotCore:12.0.0") { isTransitive = false }
    testImplementation("org.firstinspires.ftc:Hardware:12.0.0") { isTransitive = false }
    testImplementation("org.junit.jupiter:junit-jupiter:5.10.2")
}
