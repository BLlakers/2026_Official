# BL Robotics 2026

![main workflow](https://github.com/BLlakers/2026_Official/actions/workflows/main.yml/badge.svg)

This repository contains our Java/WPILib robot code with a simulation-first workflow, a swerve drivetrain, vendor motor
abstractions, and CI with test coverage. This guide explains how to set up your environment, run the sim, deploy to the
robot, and contribute.

## Current State: 2026 Season Migration (In Progress)

This codebase is currently in a **transitional state** as we migrate from the 2025 season to the 2026 season:

### What's Complete:
- ✓ **WPILib 2026 Migration** - Upgraded to 2026.1.1-beta-2 toolchain
- ✓ **PathPlanner → Choreo Migration** - Transitioning autonomous path planning frameworks
- ✓ **FuelSubsystem Implementation** - Adapted from 2026 KitBot reference implementation with team Context pattern
- ✓ **TemplateMechanism Example** - Phoenix6 TalonFX reference subsystem for new mechanism development
- ✓ **Legacy Subsystem Removal** - Removed 2025-specific mechanisms (climb, elevator, vision/Limelight)

### Architecture Pattern:
This codebase uses a **Context-based configuration pattern** inspired by the 2026 KitBot reference implementation:
- **Context Classes**: Lombok `@Builder` pattern for testable, flexible configuration (e.g., `FuelSubsystemContext`, `DrivetrainContext`)
- **Subsystems**: Accept Context objects via constructor dependency injection
- **Command Factories**: Subsystems expose command factory methods (`getIntakeCommand()`, `getLaunchCommand()`)
- **KitBot Best Practices**: SparkMaxConfig with proper reset/persist modes, voltage-based control for simple mechanisms, SmartDashboard tuning

### What's Next:
- MK5i Swerve Module integration (planned hardware upgrade)
- Additional 2026 game-specific mechanisms
- Choreo autonomous routine development
- Further refinement of subsystem implementations

For new subsystem development, refer to:
- `FuelSubsystem` - SparkMax-based roller mechanism (adapted from KitBot)
- `TemplateMechanism` - TalonFX-based mechanism baseline

---

## Table of Contents
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Project Layout](#project-layout)
- [Driver Station Controls](#driver-station-controls)
- [Running the Simulator](#running-the-simulator)
- [Deploying to the Robot](#deploying-to-the-robot)
- [Testing & Code Coverage](#testing--code-coverage)
- [Formatting & Static Checks](#formatting--static-checks)
- [Configuration & Constants](#configuration--constants)
- [Resources](#resources)
- [Contributing](#contributing)
- [License](#license)

---

## Prerequisites

- **OS:** Windows 10/11, macOS, or Linux
- **Java:** **JDK 17 (LTS)**
  ```bash
  java -version
  ```
- **IDE:** one of
    - **VS Code + WPILib Extension** (recommended)
    - IntelliJ IDEA / Eclipse (fine; use Gradle tasks)
- **Git:** for cloning & PRs
- **Gradle:** **not required** — project uses the **Gradle Wrapper** (`./gradlew`)

> Vendor libraries are managed via **vendordeps** JSON under `./vendordeps/`. GradleRIO & wrapper versions in `build.gradle` pin the WPILib toolchain.

---

## Quick Start

```bash
git clone <repo-url>
cd <repo-dir>

# First build (downloads WPILib & vendors)
./gradlew build

# Open the folder in your IDE (VS Code: install WPILib extension + Set Team Number)
```

---

## Project Layout

```
src/main/java/frc/robot/
  commands/            # Command-based routines (e.g., swervedrive/)
  subsystems/          # Subsystems (drivetrain/, fuel/, template/)
  sim/                 # Simulation helpers (SwerveModuleSim, physics)
  support/             # Vendor abstractions (sparkmax/, sensors, utils)
  Constants.java       # Global constants (units in identifiers where possible)
vendordeps/            # Vendor JSONs (REV, CTRE Phoenix6, Choreo, etc.)
```
---

## Driver Station Controls
Driver Station (3 Controllers):
```
USB 0: DRIVER CONTROLLER
├─ Swerve drive (left stick: translation, right stick: rotation)
├─ Right trigger: acceleration/gas
├─ Left trigger: half-speed mode (≥0.5 threshold)
├─ Gyro reset (B button)
└─ Wheel lock (right stick button)

USB 1: MANIPULATION/OPERATOR CONTROLLER
├─ Fuel intake (left bumper - hold)
├─ Fuel launch (right bumper - spin up → launch sequence)
└─ Fuel eject (X button - hold)

USB 2: DEBUG CONTROLLER
└─ (Reserved for testing and overrides)
```

### 1. **Driver Controller** (Channel 0)
- **Primary job:** Drive the robot using swerve drivetrain
- Left stick: Translation (forward/back, strafe left/right)
- Right stick: Rotation
- Right trigger: Acceleration multiplier
- Left trigger: Half-speed mode for precision
- B button: Reset gyro/NavX heading
- Right stick button: Toggle wheel lock (X-pattern for defense)

### 2. **Manipulation Controller** (Channel 1)
- **Primary job:** Operate fuel subsystem
- Left bumper (hold): Intake fuel into mechanism
- Right bumper (hold): Launch sequence (1 second spin-up, then launch)
- X button (hold): Eject/reverse fuel out of intake

### 3. **Debug Controller** (Channel 2)
- **Primary job:** Testing and overrides
- Currently reserved for development/testing purposes
- Used during practice for mechanism testing and troubleshooting

## Running the Simulator

```bash
./gradlew simulateJava
```

This launches **WPILib Sim GUI**.

### Keyboard / Gamepad mapping
1. Sim GUI → **Driver Station → Joysticks**: Ensure **Keyboard** is **index 0**.
2. **Configure Keyboard**:
    - **Axis 0** ← A/D (strafe left/right)
    - **Axis 1** ← W/S (forward/back)
    - (Optional) **Axis 4** ← rotation
3. In simulation we read `DriverStation.getStickAxis(0, axis)`, so the above mappings immediately drive the robot.

### Sim behavior notes
- We **reset odometry on sim enable** so field-relative starts at heading ~0°.
- **Only one pose update per loop** in sim (no double counting between `periodic()` and `simulationPeriodic()`).
- Swerve modules use `SwerveModuleState.optimize(...)` and a snappy steering loop for crisp direction changes.

---

## Deploying to the Robot

Set your team number via VS Code WPILib (“Set Team Number”) or GradleRIO config.

```bash
# On the FRC network, robot powered
./gradlew deploy
```

Then enable using the **FRC Driver Station**.

---

## Testing & Code Coverage

### JUnit 5 + Mockito

- **JUnit 5** is enabled via `useJUnitPlatform()`.
- **Mockito** (including the JUnit 5 extension) is available for mocks/spies/captors.

Example test skeleton:

```java
@ExtendWith(org.mockito.junit.jupiter.MockitoExtension.class)
class ExampleTest {
  @org.mockito.Mock SomeDep dep;

  @org.junit.jupiter.api.Test
  void works() {
    // arrange
    // act
    // assert
  }
}
```

### JaCoCo coverage in Gradle

We enforce minimum coverage and generate HTML & XML reports.

```groovy
// build.gradle (Groovy DSL)
plugins {
  id 'java'
  id 'jacoco'
}

test {
  useJUnitPlatform()
}

jacoco {
  toolVersion = "0.8.10"
}

jacocoTestReport {
  dependsOn test
  reports {
    html.required = true   // build/reports/jacoco/test/html/index.html
    xml.required  = true   // build/reports/jacoco/test/jacocoTestReport.xml
    csv.required  = false
  }
}

jacocoTestCoverageVerification {
    dependsOn test
    violationRules {
        rule {
            limit {
                counter = 'LINE'
                value   = 'COVEREDRATIO'
                minimum = 0.03 // <-- adjust threshold as we expand our test coverage
            }
        }
    }
}

check.dependsOn jacocoTestReport, jacocoTestCoverageVerification
```

### GitHub Actions CI (build, test, upload coverage)

Create `.github/workflows/ci.yml`:

```yaml
name: FRC Team 2534 CI Pipeline
on:
  push:
    branches:
      - main
      - Dev
      - Testing
  pull_request:
jobs:

  build:
    runs-on: ubuntu-22.04
    container: wpilib/roborio-cross-ubuntu:2025-22.04
    steps:
      - uses: actions/checkout@v4

      - name: Add repository to git safe directories
        run: git config --global --add safe.directory $GITHUB_WORKSPACE

      - name: Grant execute permission for gradlew
        run: chmod +x gradlew

      - name: Compile and run tests on robot code
        run: ./gradlew --no-daemon clean check jacocoTestReport jacocoTestCoverageVerification

      - name: Upload JaCoCo HTML report
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: jacoco-html
          path: build/reports/jacoco/test/html

      - name: Upload JaCoCo XML (for tooling)
        uses: actions/upload-artifact@v4
        if: always()
        with:
          name: jacoco-xml
          path: build/reports/jacoco/test/jacocoTestReport.xml
```

> If coverage drops below the configured threshold, the CI job fails and the PR turns red. Reviewers can download the HTML report artifact and open `index.html`.

---

## Formatting & Static Checks

We use **Spotless** for formatting with a Palantir config.

```bash
# Auto-format
./gradlew spotlessApply

# Build + run tests + coverage gates
./gradlew check
```

---

## Configuration & Constants

- Keep **units in identifiers** (e.g., `MAX_SPEED_MPS`, `NOMINAL_BATT_VOLTS`, `VOLTS_PER_MPS`).
- Use `*Context` classes for tunables (PID gains, gear ratios, conversion factors).
- For REV SPARK MAX **Alternate Encoder** on the real robot, configure the **data port mode + CPR** *before* selecting `kAlternateOrExternalEncoder`. In **simulation**, prefer the **primary encoder** feedback unless you fully simulate the alt encoder.

---

## Resources

### WPILib & FRC
- **WPILib Docs:** https://docs.wpilib.org/
- **WPILib Java API:** https://first.wpi.edu/wpilib/allwpilib/docs/release/java/
- **Kinematics & Odometry (Swerve):** https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/index.html
- **Robot Simulation (WPILib):** https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/index.html
- **2026 KitBot Reference:** https://github.com/wpilibsuite/allwpilib/tree/main/wpilibjExamples/src/main/java/edu/wpi/first/wpilibj/examples

### Vendor Libraries
- **REV SPARK MAX Docs:** https://docs.revrobotics.com/sparkmax/
- **CTRE Phoenix 6 (TalonFX):** https://v6.docs.ctr-electronics.com/
- **Choreo (Path Planning):** https://sleipnirgroup.github.io/Choreo/
- **navX-MXP:** https://pdocs.kauailabs.com/navx-mxp/

### Legacy (2025 Season)
- **PathPlanner:** https://pathplanner.dev/ (migrating to Choreo)
- **Limelight:** https://docs.limelightvision.io/ (removed for 2026)

---

## Contributing

- Branch from `main`.
- Keep changes focused; add/extend unit tests.
- Open a PR with:
    - **Why** + **What changed**
    - **Testing notes** (sim and/or real)
    - **Risks / required config**
- CI must pass (build, tests, coverage gates).

---

## License

This project follows the WPILib BSD license model unless otherwise noted in the repository.