android {
    namespace = "org.hivevisionftc"
    compileSdk = 35

    defaultConfig {
        minSdk = 24
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }

    testOptions {
        // The suites run on the JVM; Android framework calls (Log, etc.) return defaults instead of throwing.
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
    implementation("org.firstinspires.ftc:RobotCore:12.0.0")
    implementation("org.firstinspires.ftc:Hardware:12.0.0")
    // VisionPortal / VisionProcessor + OpenCV for the webcam track.
    implementation("org.firstinspires.ftc:Vision:12.0.0")
    implementation("androidx.annotation:annotation:1.6.0")

    testImplementation("org.junit.jupiter:junit-jupiter:5.10.2")
}
