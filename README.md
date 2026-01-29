# BL Robotics 2026

![main workflow](https://github.com/BLlakers/2026_Official/actions/workflows/main.yml/badge.svg)

This repository contains our Java/WPILib robot code with a simulation-first workflow, a swerve drivetrain, vendor motor
abstractions, and CI with test coverage. This guide explains how to set up your environment, run the sim, deploy to the
robot, and contribute.

## Current State: 2026 Season Migration (In Progress)

This codebase is currently in a **transitional state** as we migrate from the 2025 season to the 2026 season:

### What's Complete:
- ✓ **WPILib 2026 Migration** - Upgraded to 2026.1.1-beta-2 toolchain
- ✓ **PathPlanner Integration** - Using 2026-compatible PathPlanner for autonomous path planning
- ✓ **FuelSubsystem Implementation** - Adapted from 2026 KitBot reference implementation with team Context pattern
- ✓ **TemplateMechanism Example** - Phoenix6 TalonFX reference subsystem for new mechanism development
- ✓ **Legacy Subsystem Removal** - Removed 2025-specific mechanisms (climb, elevator, vision/Limelight)
- ✓ **PhotonVision Integration** - Vision subsystem with dual-camera AprilTag detection and SmartDashboard telemetry
- ✓ **Enhanced Telemetry System** - Level-aware data capture (NONE/MATCH/LAB/VERBOSE), USB auto-detection, structured type support, AdvantageScope-compatible logging for match replay

### Architecture Pattern:
This codebase uses a **Context-based configuration pattern** inspired by the 2026 KitBot reference implementation:
- **Context Classes**: Lombok `@Builder` pattern for testable, flexible configuration (e.g., `FuelSubsystemContext`, `DrivetrainContext`)
- **Subsystems**: Accept Context objects via constructor dependency injection
- **Command Factories**: Subsystems expose command factory methods (`getIntakeCommand()`, `getLaunchCommand()`)
- **KitBot Best Practices**: SparkMaxConfig with proper reset/persist modes, voltage-based control for simple mechanisms, SmartDashboard tuning

### What's Next:
- MK5i Swerve Module integration (planned hardware upgrade)
- Additional 2026 game-specific mechanisms
- PathPlanner autonomous routine development
- Further refinement of subsystem implementations

For new subsystem development, refer to:
- `FuelSubsystem` - SparkMax-based roller mechanism (adapted from KitBot)
- `TemplateMechanism` - TalonFX-based mechanism baseline
- `Drivetrain.captureTelemetry()` - Example of telemetry integration pattern

---

## Table of Contents
- [Prerequisites](#prerequisites)
- [Quick Start](#quick-start)
- [Project Layout](#project-layout)
- [Driver Station Controls](#driver-station-controls)
- [Running the Simulator](#running-the-simulator)
- [Deploying to the Robot](#deploying-to-the-robot)
- [PhotonVision Development Loop](#photonvision-development-loop)
- [Telemetry System](#telemetry-system)
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
  subsystems/          # Subsystems (drivetrain/, fuel/, vision/, template/)
  sim/                 # Simulation helpers (SwerveModuleSim, physics)
  support/             # Utilities and abstractions
    ├── Telemetry.java       # Enhanced telemetry API
    ├── TelemetryLevel.java  # Verbosity levels (NONE/MATCH/LAB/VERBOSE)
    ├── TelemetryConfig.java # Configuration with USB detection
    ├── sparkmax/            # REV SPARK MAX abstractions
    └── limelight/           # Limelight utilities (legacy)
  Constants.java       # Global constants (units in identifiers where possible)
src/main/deploy/
  telemetry.properties # Telemetry configuration (level, USB settings)
vendordeps/            # Vendor JSONs (REV, CTRE Phoenix6, PathPlanner, etc.)
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

## PhotonVision Development Loop

The **VisionSubsystem** provides AprilTag detection using PhotonVision with support for dual cameras. This section describes the complete development and validation workflow.

### Architecture Overview

The vision subsystem follows our Context-based pattern:
- **VisionSubsystem** (`src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java`) - Main subsystem class
- **VisionSubsystemContext** (`src/main/java/frc/robot/subsystems/vision/VisionSubsystemContext.java`) - Configuration with camera names and telemetry settings
- **Constants.Vision** (`src/main/java/frc/robot/Constants.java`) - Camera transforms, thresholds, and configuration constants

### Current Capabilities (Proof of Life)

The VisionSubsystem currently provides:
- ✓ **Dual Camera Support** - Front and rear cameras configured independently
- ✓ **AprilTag Detection** - Detects and logs all visible AprilTag IDs
- ✓ **SmartDashboard Telemetry** - Real-time camera status and detection data
- ✓ **Connection Monitoring** - Tracks camera connectivity status
- ✓ **Target Details** - Yaw, pitch, area, and skew for each detected tag

### SmartDashboard Telemetry Reference

When the robot is running, the following data is available in SmartDashboard/Shuffleboard:

#### System-Level
- `Vision/Status` - Overall system status string
- `Vision/TotalTagsDetected` - Combined tag count from both cameras

#### Per-Camera (Front and Rear)
- `Vision/FrontCamera/Connected` - Connection status (boolean)
- `Vision/FrontCamera/TargetCount` - Number of detected tags
- `Vision/FrontCamera/DetectedTags` - Comma-separated list of tag IDs (e.g., "[4, 7, 14]")
- `Vision/FrontCamera/TimestampSeconds` - Result timestamp in seconds
- `Vision/FrontCamera/BestTargetID` - Fiducial ID of the best target
- `Vision/FrontCamera/BestTargetYaw` - Yaw angle to best target

#### Verbose Logging (if enabled)
When `enableVerboseLogging` is true in VisionSubsystemContext:
- `Vision/FrontCamera/Tag{ID}/Yaw` - Yaw angle for specific tag
- `Vision/FrontCamera/Tag{ID}/Pitch` - Pitch angle for specific tag
- `Vision/FrontCamera/Tag{ID}/Area` - Target area percentage
- `Vision/FrontCamera/Tag{ID}/Skew` - Target skew angle

### Hardware Setup

#### 1. Camera Installation
1. Mount cameras on robot (front and rear recommended)
2. Connect cameras to roboRIO via USB or Ethernet
3. Ensure cameras have clear view of field AprilTags
4. Note physical mounting positions for calibration

#### 2. PhotonVision Configuration
On each camera's coprocessor (Raspberry Pi, Orange Pi, etc.):

1. **Install PhotonVision** (if not already installed)
   - Download latest release from [photonvision.org](https://photonvision.org)
   - Flash to coprocessor following official docs

2. **Access PhotonVision UI**
   - Navigate to `http://photonvision.local:5800` or `http://<coprocessor-ip>:5800`

3. **Configure Camera Settings**
   - Go to **Settings** → **Cameras**
   - Name cameras to match Constants.Vision:
     - Front camera: `photonvision-front`
     - Rear camera: `photonvision-rear`
   - **CRITICAL:** Camera names MUST match exactly

4. **Configure AprilTag Pipeline**
   - Create new pipeline (type: AprilTag)
   - Set tag family to **36h11** (2024/2025/2026 FRC standard)
   - Enable **Multi-target mode** to detect multiple tags simultaneously
   - Tune exposure/brightness for consistent detection

5. **Set Static IP (Recommended)**
   - Configure coprocessor with static IP on robot network
   - Example: `10.TE.AM.11` for front, `10.TE.AM.12` for rear
   - Update mDNS hostname if using `photonvision.local`

### Development Workflow

#### Step 1: Initial Integration Test

**Goal:** Verify cameras connect and PhotonVision NetworkTables communication works

```bash
# Deploy robot code
./gradlew deploy

# Open Driver Station and enable robot (TeleOp or Test mode)
```

**Open SmartDashboard/Shuffleboard:**
1. Check `Vision/FrontCamera/Connected` and `Vision/RearCamera/Connected`
   - Both should show `true` (green)
   - If `false`: Check PhotonVision coprocessor power, network connection, camera names

2. Check `Vision/Status`
   - Should show "No Targets Detected" or "Tracking" when tags are visible
   - If "No Cameras Connected": Verify camera names in PhotonVision UI match Constants.Vision

**Troubleshooting:**
- **Camera not connecting:**
  - Verify PhotonVision service is running on coprocessor
  - Check network connectivity: `ping <coprocessor-ip>` from Driver Station
  - Confirm camera names match exactly (case-sensitive)
  - Restart PhotonVision service
- **NetworkTables not updating:**
  - Verify robot and coprocessor are on same network
  - Check PhotonVision → Settings → NetworkTables client (should show roboRIO IP)

#### Step 2: AprilTag Detection Validation

**Goal:** Verify cameras detect AprilTags and log correct IDs

**Setup:**
1. Print 2026 FRC AprilTags (download from [firstfrc.blob.core.windows.net](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/AprilTags.pdf))
2. Position tags in camera field of view
3. Use known tag IDs (e.g., Blue Speaker Center = ID 7, Red Speaker Center = ID 4)

**Test Procedure:**
1. Enable robot in TeleOp mode
2. Position Tag ID 7 in front camera view
3. **Observe SmartDashboard:**
   - `Vision/FrontCamera/TargetCount` should increase to `1` or more
   - `Vision/FrontCamera/DetectedTags` should show `"[7]"`
   - `Vision/FrontCamera/BestTargetID` should show `7`

4. Move tag to rear camera view
5. **Observe SmartDashboard:**
   - `Vision/RearCamera/TargetCount` should increase
   - `Vision/RearCamera/DetectedTags` should show `"[7]"`

6. Position multiple tags (IDs 4, 7, 14) in view simultaneously
7. **Verify multi-target detection:**
   - `DetectedTags` should show all IDs: `"[4, 7, 14]"`
   - `TotalTagsDetected` should show `3`

**Expected Results:**
- ✓ Correct tag IDs logged for each camera
- ✓ Multiple tags detected simultaneously
- ✓ `BestTargetYaw` updates as tag moves laterally
- ✓ Timestamp values update continuously

**Troubleshooting:**
- **Tags not detected:**
  - Check lighting conditions (AprilTags need good contrast)
  - Verify pipeline is active in PhotonVision UI (green indicator)
  - Adjust exposure/brightness in PhotonVision camera settings
  - Ensure tags are printed at correct size (8.5" for 2026 field tags)
- **Wrong tag IDs logged:**
  - Verify tag family setting (must be 36h11)
  - Check tag print quality (damaged/blurry tags may misidentify)
- **Intermittent detection:**
  - Increase camera exposure slightly
  - Reduce coprocessor CPU load (close unnecessary processes)
  - Check for camera focus issues

#### Step 3: Field Testing and Calibration

**Goal:** Test on practice field and calibrate camera transforms

**On Practice Field:**
1. Deploy robot code and enable
2. Drive robot around field
3. **Monitor SmartDashboard continuously:**
   - Verify tags detected from various distances (0.5m to 4m+)
   - Check which camera sees which tags (front vs rear)
   - Note detection range limits

4. **Log observations:**
   - Maximum reliable detection distance
   - Which field positions provide best multi-tag visibility
   - Any blind spots or occlusion issues

**Camera Transform Calibration:**

The VisionSubsystem needs accurate camera-to-robot transforms for future pose estimation.

1. **Measure physical camera positions:**
   - X: Forward distance from robot center (meters, positive = forward)
   - Y: Lateral distance from robot center (meters, positive = left)
   - Z: Height above robot center (meters, positive = up)

2. **Measure camera orientations:**
   - Roll: Rotation around X-axis (radians, typically 0)
   - Pitch: Rotation around Y-axis (radians, angle camera is tilted)
   - Yaw: Rotation around Z-axis (radians, 0 = forward, π = backward)

3. **Update Constants.Vision in Constants.java:**

```java
public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(
    new Translation3d(0.3, 0.0, 0.5),  // Example: 30cm forward, centered, 50cm up
    new Rotation3d(0, -Math.toRadians(15), 0)  // Example: 15° pitched down
);

public static final Transform3d REAR_CAMERA_TO_ROBOT = new Transform3d(
    new Translation3d(-0.3, 0.0, 0.5),  // Example: 30cm rearward, centered, 50cm up
    new Rotation3d(0, -Math.toRadians(15), Math.PI)  // Example: 15° pitched down, 180° yaw
);
```

4. **Redeploy and validate:**
   - These transforms will be used for SwerveDrivePoseEstimator integration (next phase)

#### Step 4: Integration Validation

**Goal:** Confirm VisionSubsystem integrates cleanly with existing robot code

**Test Checklist:**
- [ ] Robot boots successfully with VisionSubsystem enabled
- [ ] No NetworkTables conflicts (check for error messages in Driver Station console)
- [ ] Swerve drivetrain still operates normally with vision running
- [ ] No performance degradation (check robot loop timing in Driver Station)
- [ ] All subsystems (Drivetrain, FuelSubsystem, VisionSubsystem) show in SmartDashboard

**Autonomous Testing:**
- Run existing autonomous routines with VisionSubsystem active
- Verify no interference with PathPlanner path following
- Check that vision data is available throughout autonomous period

### Configuration Reference

#### Camera Names
Defined in `Constants.Vision` and must match PhotonVision UI configuration:
```java
public static final String PHOTON_CAMERA_FRONT = "photonvision-front";
public static final String PHOTON_CAMERA_REAR = "photonvision-rear";
```

#### Tunable Parameters
In `VisionSubsystemContext.defaults()`:
- `enableVerboseLogging` - Logs detailed per-tag telemetry (default: true)
- `enableCameraTelemetry` - Enables camera-level telemetry (default: true)

In `Constants.Vision` (for future pose estimation):
- `POSE_AMBIGUITY_THRESHOLD` - Reject targets with ambiguity > 0.2
- `MAX_POSE_ESTIMATION_DISTANCE` - Max distance (meters) to trust vision

### Next Steps: Pose Estimation

The current VisionSubsystem provides proof-of-life functionality. Future enhancements will include:

1. **SwerveDrivePoseEstimator Integration**
   - Fuse vision measurements with drivetrain odometry
   - Use `addVisionMeasurement()` with timestamp and standard deviations
   - Implement ambiguity filtering and multi-tag validation

2. **Automatic Target Alignment**
   - Commands to align robot to AprilTags (e.g., speaker, amp)
   - PID controllers for yaw/distance control using vision feedback

3. **Autonomous Localization**
   - Reset odometry using vision at autonomous start
   - Continuous pose correction during auto routines

4. **Field-Relative Corrections**
   - Periodic heading correction using tag orientations
   - Handle alliance color flipping (red vs blue)

### Additional Resources

- **PhotonVision Docs:** https://docs.photonvision.org/
- **PhotonLib Java API:** https://docs.photonvision.org/en/latest/docs/programming/photonlib/index.html
- **2026 AprilTag Layout:** https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/AprilTags.pdf
- **WPILib Pose Estimation:** https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html

---

## Telemetry System

The **Telemetry** system provides high-fidelity data capture for post-match analysis, debugging, and performance tuning. Data is automatically logged to USB storage (when available) in WPILib's `.wpilog` format, which can be replayed in **AdvantageScope** for full match visualization.

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Robot Code                                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │ Drivetrain  │  │   Vision    │  │    Fuel     │  │  Commands   │    │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘    │
│         │                │                │                │            │
│         └────────────────┴────────────────┴────────────────┘            │
│                                   │                                      │
│                          ┌────────▼────────┐                            │
│                          │    Telemetry    │  ← Single instrumentation  │
│                          │  (Level-aware)  │    point for all data      │
│                          └────────┬────────┘                            │
└───────────────────────────────────┼──────────────────────────────────────┘
                                    │
              ┌─────────────────────┴─────────────────────┐
              │                                           │
       ┌──────▼──────┐                            ┌───────▼───────┐
       │  DataLog    │                            │ NetworkTables │
       │ (.wpilog)   │                            │  (real-time)  │
       └──────┬──────┘                            └───────────────┘
              │
       ┌──────▼──────┐
       │ USB Stick   │  ← Auto-detected at /media/sda1/FRC_LOGS/
       │ (portable)  │
       └──────┬──────┘
              │
       ┌──────▼──────┐
       │ AdvantageScope │  ← Replay, visualize, analyze
       └─────────────────┘
```

### File Structure

```
src/main/java/frc/robot/support/
├── Telemetry.java          # Main telemetry API (record, event, error)
├── TelemetryLevel.java     # Verbosity levels (NONE, MATCH, LAB, VERBOSE)
└── TelemetryConfig.java    # Configuration with USB detection

src/main/deploy/
└── telemetry.properties    # Deploy-time configuration
```

### Telemetry Levels

| Level | Use Case | Data Rate | What Gets Logged |
|-------|----------|-----------|------------------|
| `NONE` | Emergency/Competition backup | 0 Hz | Nothing (silent mode) |
| `MATCH` | Competition matches | ~20 Hz | Essential: pose, commands, errors |
| `LAB` | Practice/testing | ~50 Hz | MATCH + motor currents, sensor raw values |
| `VERBOSE` | Deep debugging | ~100+ Hz | LAB + PID internals, all signals |

### Quick Start

#### 1. Automatic Initialization (Already Configured)

Telemetry is automatically initialized in `Robot.java`:

```java
@Override
public void robotInit() {
    // Loads config from telemetry.properties or uses defaults
    Telemetry.initialize(TelemetryConfig.fromDeployDirectory());
    // ...
}

@Override
public void robotPeriodic() {
    // Captures all registered subsystems
    Telemetry.periodic();
    // ...
}
```

#### 2. Instrument Your Subsystems

Register subsystems for automatic capture:

```java
// In subsystem constructor
public MySubsystem(MySubsystemContext context) {
    // ... initialization ...

    // Register for automatic telemetry capture
    Telemetry.registerSubsystem("MySubsystem", this::captureTelemetry);
}

// Capture method - called automatically by Telemetry.periodic()
private void captureTelemetry(String prefix) {
    // MATCH level - essential data (always captured at MATCH+)
    Telemetry.record(prefix + "/Position", getPosition(), TelemetryLevel.MATCH);
    Telemetry.record(prefix + "/Velocity", getVelocity(), TelemetryLevel.MATCH);

    // LAB level - detailed data (captured at LAB+)
    Telemetry.record(prefix + "/MotorCurrent", motor.getOutputCurrent(), TelemetryLevel.LAB);
    Telemetry.record(prefix + "/MotorTemp", motor.getMotorTemperature(), TelemetryLevel.LAB);

    // VERBOSE level - debug data (only at VERBOSE)
    Telemetry.record(prefix + "/PID/Error", pidController.getPositionError(), TelemetryLevel.VERBOSE);
    Telemetry.record(prefix + "/PID/Setpoint", pidController.getSetpoint(), TelemetryLevel.VERBOSE);
}
```

#### 3. Log Events and Errors

```java
// Significant events (always captured regardless of level)
Telemetry.event("Intake/Acquired", "Coral detected at " + Timer.getFPGATimestamp());
Telemetry.event("Auto/PathStarted", "Running path: " + pathName);

// Errors (always captured + shown in DriverStation)
Telemetry.error("Launcher", "Overcurrent detected", motorCurrent);
Telemetry.error("Vision", "Camera disconnected", exception);
```

#### 4. Structured Types (AdvantageScope-Ready)

Native WPILib struct support for visualization:

```java
// Pose2d - appears as field visualization in AdvantageScope
Telemetry.record("Drivetrain/Pose", getPose2d(), TelemetryLevel.MATCH);

// ChassisSpeeds - velocity vectors
Telemetry.record("Drivetrain/Speeds", getChassisSpeeds(), TelemetryLevel.MATCH);

// SwerveModuleState[] - swerve visualizer
Telemetry.record("Drivetrain/ModuleStates", getSwerveModuleStates(), TelemetryLevel.LAB);

// Pose2d[] - trajectory visualization
Telemetry.recordPoses("Auto/PlannedPath", trajectoryPoses, TelemetryLevel.LAB);
```

### Configuration

#### Deploy-Time Configuration (telemetry.properties)

Located at `src/main/deploy/telemetry.properties`:

```properties
# Telemetry verbosity level (NONE, MATCH, LAB, VERBOSE)
telemetry.level=LAB

# Enable USB stick detection for log storage
telemetry.usb.enabled=true

# Log NetworkTables data for full replay in AdvantageScope
telemetry.networktables.enabled=true

# Custom log path (optional, overrides USB detection)
# telemetry.path=/home/lvuser/custom_logs
```

#### Runtime Level Changes

Change level during operation via NetworkTables:

1. Open SmartDashboard/Shuffleboard
2. Navigate to `Telemetry/Level`
3. Change value to: `NONE`, `MATCH`, `LAB`, or `VERBOSE`

Or programmatically:
```java
Telemetry.setLevel(TelemetryLevel.VERBOSE);  // For debugging
Telemetry.setLevel(TelemetryLevel.MATCH);    // For competition
```

### USB Stick Setup

#### Preparing the USB Stick

1. **Format:** FAT32 or ext4 (FAT32 recommended for cross-platform)
2. **Capacity:** 8GB+ recommended (typical match log: 10-20MB)
3. **Speed:** USB 2.0 minimum, USB 3.0 preferred

#### On the roboRIO

1. Insert USB stick into roboRIO USB port
2. System auto-creates `FRC_LOGS/` directory
3. Logs are written with timestamped filenames: `FRC_YYYYMMDD_HHMMSS.wpilog`

#### Auto-Detection Paths (checked in order)

| Path | Description |
|------|-------------|
| `/media/sda1/FRC_LOGS` | Primary USB mount on roboRIO 2 |
| `/U/FRC_LOGS` | Alternate USB mount point |
| `/home/lvuser/logs` | Fallback (internal storage) |

### Match Analysis Workflow

#### Scenario: Subsystem Failure During Competition

> "During qualification match 47, our launcher failed at 1:45 into the match. The driver pressed the launch button, but nothing happened."

#### Step 1: Retrieve Log (2 minutes)

```
PIT AREA
├── Remove USB stick from roboRIO
├── Insert into pit laptop
└── Navigate to: FRC_LOGS/FRC_20240315_142500.wpilog
```

#### Step 2: Open in AdvantageScope (1 minute)

```
AdvantageScope
├── File → Open Log → Select .wpilog file
├── Data tree shows all logged signals:
│   ├── Drivetrain/
│   │   ├── Pose (Pose2d) → Field visualization
│   │   ├── Speeds (ChassisSpeeds)
│   │   └── ModuleStates (SwerveModuleState[])
│   ├── Launcher/
│   │   ├── Voltage
│   │   ├── Current
│   │   ├── RPM
│   │   └── Error (event log)
│   └── Match/
│       ├── Mode (Auto/Teleop/Disabled)
│       └── Time
```

#### Step 3: Navigate to Failure Point (3 minutes)

1. Use timeline to navigate to **1:45** (failure time)
2. Add signals to graph:
   - `Launcher/Voltage` - Was voltage commanded?
   - `Launcher/Current` - Motor current draw
   - `Launcher/RPM` - Actual motor speed
3. Look for the **event marker**: `Launcher/Error`

#### Step 4: Identify Root Cause

```
Timeline: 1:44:800 ──────────────────────── 1:46:200

Launcher/Voltage
12V ┤                    ┌────────────────────
    │                    │ ← Voltage commanded correctly
 0V ┤────────────────────┘

Launcher/Current
40A ┤                    ┌──┐ ← Current spike (STALL!)
20A ┤                    │  │
 0A ┤────────────────────┘  └──────

Launcher/RPM
3000 ┤
   0 ┤────────────────────────────── ← RPM stays at 0!

Launcher/Error
─────────────────────────X─────────────────────
                         └── "Overcurrent: 42.3A"
```

**Diagnosis:** Motor stalled (12V applied, 42A current, 0 RPM = mechanical jam)

#### Step 5: Replay in Simulation (Optional)

AdvantageScope supports synchronized playback:
- **Field2d view:** See robot position at failure
- **Swerve visualizer:** Check drivetrain state
- **Slow motion:** Step through critical moments

### Telemetry API Reference

#### Recording Data

```java
// Primitives (with level)
Telemetry.record(String key, double value, TelemetryLevel level);
Telemetry.record(String key, int value, TelemetryLevel level);
Telemetry.record(String key, boolean value, TelemetryLevel level);
Telemetry.record(String key, String value, TelemetryLevel level);
Telemetry.record(String key, double[] values, TelemetryLevel level);

// Structured types (AdvantageScope visualization)
Telemetry.record(String key, Pose2d pose, TelemetryLevel level);
Telemetry.record(String key, Pose3d pose, TelemetryLevel level);
Telemetry.record(String key, Rotation2d rotation, TelemetryLevel level);
Telemetry.record(String key, ChassisSpeeds speeds, TelemetryLevel level);
Telemetry.record(String key, SwerveModuleState[] states, TelemetryLevel level);
Telemetry.recordPoses(String key, Pose2d[] poses, TelemetryLevel level);

// Legacy API (defaults to MATCH level)
Telemetry.record(String key, double value);
Telemetry.record(String key, String value);
```

#### Events and Errors

```java
// Events (always captured)
Telemetry.event(String key, String description);
Telemetry.event(String key);  // Auto-timestamps

// Errors (always captured + DriverStation notification)
Telemetry.error(String subsystem, String message);
Telemetry.error(String subsystem, String message, double value);
Telemetry.error(String subsystem, String message, Throwable cause);
```

#### Subsystem Registration

```java
// Register for automatic capture
Telemetry.registerSubsystem(String name, Consumer<String> captureFunction);

// Unregister (if needed)
Telemetry.unregisterSubsystem(String name);
```

#### Utility Methods

```java
// Level management
TelemetryLevel Telemetry.getCurrentLevel();
void Telemetry.setLevel(TelemetryLevel level);

// Status
boolean Telemetry.isInitialized();

// Lifecycle
void Telemetry.initialize(TelemetryConfig config);
void Telemetry.periodic();  // Call in robotPeriodic()
void Telemetry.shutdown();  // Called automatically in Robot.close()
```

### SmartDashboard/NetworkTables Reference

When telemetry is active, the following entries are published:

#### System Status
- `Telemetry/Level` - Current verbosity level (editable)
- `Telemetry/Initialized` - Initialization status
- `Telemetry/LogPath` - Where logs are being written

#### Match Context
- `Match/Mode` - Current mode (Disabled/Auto/Teleop/Test)
- `Match/Time` - Match time remaining
- `Match/Alliance` - Red/Blue/Unknown

#### Per-Subsystem (example: Drivetrain)
- `Drivetrain/Pose` - Current robot pose
- `Drivetrain/Speeds` - Chassis velocities
- `Drivetrain/Heading` - Robot heading
- `Drivetrain/ModuleStates/Current` - Actual swerve states
- `Drivetrain/ModuleStates/Desired` - Commanded swerve states
- `Drivetrain/Error` - Error messages (if any)

### Best Practices

#### 1. Choose Appropriate Levels

```java
// MATCH: Data you NEED for competition analysis
Telemetry.record("Arm/Position", getPosition(), TelemetryLevel.MATCH);

// LAB: Data useful for tuning and practice
Telemetry.record("Arm/MotorCurrent", getCurrent(), TelemetryLevel.LAB);

// VERBOSE: Data only needed for deep debugging
Telemetry.record("Arm/PID/Derivative", pidD, TelemetryLevel.VERBOSE);
```

#### 2. Use Consistent Key Naming

```java
// Good: Hierarchical, descriptive
"Drivetrain/FL/DriveMotor/Current"
"Intake/State"
"Auto/PathFollowing/Error"

// Avoid: Flat, ambiguous
"flCurrent"
"state"
"error"
```

#### 3. Log Events at State Transitions

```java
public void startIntake() {
    Telemetry.event("Intake/Started");
    // ...
}

public void stopIntake() {
    Telemetry.event("Intake/Stopped", "reason=" + stopReason);
    // ...
}
```

#### 4. Include Context in Errors

```java
// Good: Includes relevant values
Telemetry.error("Launcher", "Overcurrent detected", motorCurrent);
Telemetry.error("Vision", "Tag ambiguity too high", ambiguity);

// Avoid: Vague messages
Telemetry.error("Launcher", "Error occurred");
```

### Troubleshooting

#### Logs Not Appearing on USB

1. **Check USB detection:**
   - Look at `Telemetry/LogPath` in SmartDashboard
   - Should show `/media/sda1/FRC_LOGS` if USB detected
2. **Verify USB is mounted:**
   - SSH to roboRIO: `ssh admin@roborio-XXXX-frc.local`
   - Run: `ls /media/sda1`
3. **Check USB format:** Must be FAT32 or ext4
4. **Check permissions:** `FRC_LOGS` directory should be writable

#### Log Files Too Large

1. Reduce telemetry level: `LAB` → `MATCH`
2. Remove high-frequency VERBOSE-level records
3. Use USB with more capacity
4. Clean old logs periodically

#### AdvantageScope Can't Open Log

1. Ensure `.wpilog` extension (not `.log`)
2. Download latest AdvantageScope version
3. Check log file isn't corrupted (partial write during power loss)

### Additional Resources

- **AdvantageScope:** https://github.com/Mechanical-Advantage/AdvantageScope
- **WPILib Data Logging:** https://docs.wpilib.org/en/stable/docs/software/telemetry/datalog.html
- **NetworkTables:** https://docs.wpilib.org/en/stable/docs/software/networktables/index.html

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
- **PathPlanner:** https://pathplanner.dev/
- **navX-MXP:** https://pdocs.kauailabs.com/navx-mxp/
- **PhotonVision:** https://docs.photonvision.org/

### Legacy (2025 Season)
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