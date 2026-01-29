# Robot Simulation Guide

## Overview

This codebase follows a **simulation-first development approach**. The robot's subsystems are designed to work seamlessly in both simulation and real hardware environments, enabling development, testing, and validation before physical robot construction.

## Why Simulation-First?

- **Early Development**: Begin software development before the robot is physically built
- **Rapid Iteration**: Test code changes instantly without deploying to hardware
- **Safe Experimentation**: Try new algorithms and strategies without risk of damage
- **Continuous Testing**: Automated testing in CI/CD pipelines
- **Driver Training**: Practice driving and autonomous routines before competitions
- **Parallel Workflows**: Software and build teams can work independently

## Architecture

### System Overview

The robot code is organized into three main layers:

```
┌─────────────────────────────────────────────────────┐
│                    Robot.java                       │
│              (WPILib Entry Point)                   │
└─────────────────────┬───────────────────────────────┘
                      │
                      ▼
┌─────────────────────────────────────────────────────┐
│                RobotContainer.java                  │
│          (Subsystem Instantiation & Wiring)         │
└──┬──────────────┬──────────────┬────────────────┬───┘
   │              │              │                │
   ▼              ▼              ▼                ▼
┌──────────┐ ┌──────────┐ ┌────────────┐  ┌──────────┐
│Drivetrain│ │  Vision  │ │    Fuel    │  │   LED    │
│Subsystem │ │Subsystem │ │ Subsystem  │  │  Strand  │
└──────────┘ └──────────┘ └────────────┘  └──────────┘
```

### Context-Based Dependency Injection

Each subsystem uses a **Context** class for configuration:

```java
// Subsystem construction pattern
VisionSubsystem visionSubsystem = new VisionSubsystem(
    VisionSubsystemContext.defaults(),  // Configuration
    driveTrain,                          // Dependency
    driveTrain::addVisionMeasurement     // Callback
);
```

**Why Context pattern?**
- All configuration in one place (no scattered constants)
- Easy to override defaults for testing
- Builder pattern for flexible configuration
- Type-safe dependency injection

**Example**:
```java
VisionSubsystemContext customContext = VisionSubsystemContext.builder()
    .frontCameraName("custom-camera")
    .cameraFps(60)
    .singleTagStdDevFactor(3.0)
    .build();
```

### Real vs Simulation Pattern

Our codebase uses conditional instantiation to support both real and simulated hardware:

```java
// Example from SwerveModuleContext
private static TeamSparkMax createMotor(int canId, MotorType type) {
    return (Robot.isReal())
        ? new TeamSparkMaxImpl(canId, type)      // Real hardware
        : new TeamSparkMaxSimImpl(canId, type);  // Simulated hardware
}
```

**In VisionSubsystem**:
```java
if (RobotBase.isSimulation() && context.isEnableSimulation()) {
    initializeSimulation();  // Only runs in sim
}
```

This pattern ensures:
- **Zero code changes** when switching between sim and real
- **Consistent behavior** across both environments
- **Realistic physics** in simulation

### Data Flow Architecture

#### Periodic Loop Execution

WPILib's CommandScheduler runs subsystem methods in this order:

```
Robot Start
    │
    ├─> robotInit()
    │       └─> RobotContainer constructor
    │               ├─> Creates Drivetrain
    │               ├─> Creates VisionSubsystem (passes drivetrain ref)
    │               └─> Wires up callbacks
    │
    └─> Every 20ms loop:
            ├─> robotPeriodic()
            │       └─> CommandScheduler.run()
            │               ├─> Drivetrain.periodic()
            │               ├─> VisionSubsystem.periodic()
            │               ├─> FuelSubsystem.periodic()
            │               └─> LedStrand.periodic()
            │
            └─> simulationPeriodic() [SIM ONLY]
                    └─> CommandScheduler.run() (subsystem sims)
                            ├─> Drivetrain.simulationPeriodic()
                            └─> VisionSubsystem.simulationPeriodic()
```

**Key Points**:
- `periodic()` runs in BOTH real and sim (always)
- `simulationPeriodic()` runs ONLY in sim
- Loop time budget: 20ms (0.02s)
- Loop overruns trigger warnings

### Subsystem Simulation Support

#### Drivetrain Simulation

**Location**: `Drivetrain.simulationPeriodic()` (lines 557-634)

**Features**:
- Physics-based swerve module simulation using `DCMotorSim`
- Realistic voltage-to-velocity conversion
- Simulated gyro integration
- Pose estimation with odometry
- Field2d visualization

**How it works**:
1. Converts desired module states to motor voltages
2. Updates `SwerveModuleSim` physics for each module
3. Calculates actual chassis speeds from simulated module states
4. Integrates heading from angular velocity
5. Updates `SwerveDrivePoseEstimator` with simulated measurements
6. Visualizes robot pose on Field2d

#### Vision Simulation

**Location**: `VisionSubsystem.java`

**Features**:
- Dual camera simulation (front/rear)
- AprilTag detection based on robot pose
- Realistic camera properties (FOV, latency, calibration error)
- PhotonPoseEstimator integration for localization
- Dynamic standard deviation calculation
- Field2d AprilTag wireframe visualization

**Detailed Data Flow**:

```
VisionSubsystem.simulationPeriodic() [ONLY in sim, every 20ms]
    │
    ├─> Get current robot pose from drivetrain
    │       drivetrain.getPose2dEstimator()
    │
    └─> Update vision simulation with robot pose
            visionSim.update(robotPose)
                │
                ├─> PhotonCameraSim (front) generates detections
                │   └─> Checks which AprilTags are visible:
                │       - Within camera FOV (90°)
                │       - Within detection range (~6m)
                │       - Not occluded
                │   └─> Publishes to NetworkTables (like real camera)
                │
                └─> PhotonCameraSim (rear) generates detections
                    └─> Same process for rear camera

VisionSubsystem.periodic() [ALWAYS runs, every 20ms]
    │
    ├─> Get latest camera results (via NetworkTables)
    │       frontCamera.getLatestResult()
    │       rearCamera.getLatestResult()
    │
    ├─> Log detected AprilTags to SmartDashboard
    │       processAndLogTargets()
    │
    └─> updatePoseEstimation()
            │
            ├─> For each camera with detections:
            │   │
            │   └─> processCamera()
            │           │
            │           ├─> PhotonPoseEstimator.update(result)
            │           │       │
            │           │       └─> Uses MULTI_TAG_PNP_ON_COPROCESSOR strategy
            │           │           - Combines all visible tags
            │           │           - Solves for robot pose using PnP algorithm
            │           │           - Returns EstimatedRobotPose (or empty)
            │           │
            │           ├─> shouldRejectEstimate()
            │           │   - Check if tags too far (>4m)
            │           │   - Check if ambiguity too high (>0.2)
            │           │
            │           ├─> calculateVisionStdDevs()
            │           │   - Count visible tags
            │           │   - Calculate average distance
            │           │   - Set confidence: more tags + closer = higher trust
            │           │   - Return Matrix<N3, N1> [x_stddev, y_stddev, theta_stddev]
            │           │
            │           └─> Send to drivetrain via callback:
            │               visionMeasurementConsumer.accept(
            │                   pose, timestamp, stdDevs
            │               )
            │                   │
            │                   └─> Drivetrain.addVisionMeasurement()
            │                           │
            │                           └─> SwerveDrivePoseEstimator.addVisionMeasurement()
            │                               - Fuses vision with odometry
            │                               - Weighs by standard deviations
            │                               - Updates robot pose estimate
            └─> Result: Robot pose continuously corrected by vision
```

**Key Insight**: The vision system runs in TWO parts:
1. **simulationPeriodic()**: Generates fake camera data (sim only)
2. **periodic()**: Processes camera data (both real and sim)

This means the same processing code works for both simulated and real cameras!

### Callback Pattern (Functional Interface)

The VisionSubsystem doesn't directly depend on the Drivetrain class. Instead, it uses a **callback** to send vision measurements:

```java
@FunctionalInterface
public interface VisionMeasurementConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
}
```

**In RobotContainer**:
```java
VisionSubsystem visionSubsystem = new VisionSubsystem(
    VisionSubsystemContext.defaults(),
    driveTrain,                          // For reading current pose
    driveTrain::addVisionMeasurement     // Callback using method reference
);
```

**Why use callbacks?**
- **Loose coupling**: VisionSubsystem doesn't know about Drivetrain internals
- **Testable**: Easy to provide mock callbacks in unit tests
- **Flexible**: Could send measurements to multiple consumers
- **Clear contract**: Function signature defines exact data flow

**How it works**:
1. VisionSubsystem calculates a vision pose estimate
2. It calls `visionMeasurementConsumer.accept(pose, timestamp, stdDevs)`
3. This invokes `drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)`
4. Drivetrain updates its pose estimator

This is the **Observer pattern** using Java functional interfaces.

### Performance Optimizations

**Problem**: Camera operations like `isConnected()` were slow in simulation, causing loop overruns (>20ms).

**Solution**: Optimized VisionSubsystem.periodic():

```java
// Skip expensive isConnected() calls in simulation
boolean isSimulation = RobotBase.isSimulation() && visionSim != null;
boolean frontConnected = isSimulation || frontCamera.isConnected();

// Camera streaming disabled in sim to avoid CameraServer handle errors
frontCameraSim.enableRawStream(false);
frontCameraSim.enableProcessedStream(false);
```

**Result**: VisionSubsystem.periodic() runs in <5ms instead of 127ms.

**Lesson**: Always profile simulation performance and optimize hot paths.

### PathPlanner Integration (Optional)

**Current State**: PathPlanner AutoBuilder is **not configured** because there are no PathPlanner settings files.

**What this means**:
- ✅ Simulation works fine
- ✅ Manual driving works
- ❌ PathPlanner autonomous routines unavailable
- ⚠️ AutoChooser shows only "None" option

**Architecture decision**:
```java
// DrivetrainContext.getRobotConfig()
try {
    return Optional.of(RobotConfig.fromGUISettings());
} catch (IOException | ParseException e) {
    return Optional.empty();  // Gracefully fail
}

// Drivetrain constructor
context.getRobotConfig().ifPresentOrElse(
    robotConfig -> { /* Configure AutoBuilder */ },
    () -> { System.out.println("WARNING: AutoBuilder not configured"); }
);

// RobotContainer.buildAutoChooserSafe()
try {
    return AutoBuilder.buildAutoChooser();
} catch (RuntimeException e) {
    // Fallback to basic chooser with Commands.none()
    SendableChooser<Command> chooser = new SendableChooser<>();
    chooser.setDefaultOption("None", Commands.none());
    return chooser;
}
```

**Why optional**:
- Simulation-first workflow doesn't require autonomous paths initially
- Vision testing works independently of path planning
- When ready for autonomous, create PathPlanner settings via GUI
- No code changes needed when PathPlanner is added later

**To enable PathPlanner**:
1. Open PathPlanner GUI
2. Create robot configuration (dimensions, constraints)
3. Design paths and autonomous routines
4. Settings auto-save to `deploy/pathplanner/`
5. Restart simulation - AutoBuilder will detect settings

**Lesson**: Make dependencies optional when possible to enable incremental development.

## Running Simulation

### Prerequisites

- WPILib 2026.1.1 installed
- VS Code with WPILib extension
- Java 17+

### Start Simulation

1. **Open project** in VS Code
2. **Press Ctrl+Shift+P** (Cmd+Shift+P on Mac)
3. **Type**: "WPILib: Simulate Robot Code"
4. **Select**: "Sim GUI" or "Desktop" mode
5. **Click**: "Start Simulation"

Alternatively, use Gradle:
```bash
./gradlew simulateJava
```

### Simulation GUI

The Simulation GUI provides:
- **Robot State**: Enable/disable, select mode (teleop/auto/test)
- **System Inputs**: Joysticks, timing control
- **Simulation I/O**: Gyro, motors, encoders
- **Field2d**: Visual representation of robot pose
- **NetworkTables**: Live telemetry data

## Vision Simulation Setup

### Camera Configuration

All vision configuration is defined in `VisionSubsystemContext` as builder defaults. You can override any parameter when creating the context:

```java
VisionSubsystemContext customContext = VisionSubsystemContext.builder()
    .frontCameraName("my-front-camera")
    .frontCameraToRobot(new Transform3d(
        new Translation3d(0.5, 0.0, 0.3),  // 0.5m forward, 0.3m up
        new Rotation3d(0, 0, 0)
    ))
    .cameraFps(60)  // Override FPS to 60
    .build();
```

**Default camera transforms** (TODO: Calibrate from CAD):
- Front: `(0, 0, 0)` position, `(0, 0, 0)` rotation
- Rear: `(0, 0, 0)` position, `(0, 0, π)` rotation (180° yaw)

**Important**: These transforms must be calibrated to match your physical robot's camera mounting positions.

### Camera Properties

Simulation uses realistic camera properties (all configurable via `VisionSubsystemContext`):

| Property | Default Value | Context Field | Description |
|----------|---------------|---------------|-------------|
| Resolution | 960x720 | `cameraResolutionWidth/Height` | Camera image dimensions |
| FOV | 90° | `cameraFovDegrees` | Field of view |
| FPS | 30 | `cameraFps` | Frames per second |
| Latency | 50ms ± 15ms | `cameraAvgLatencyMs/StddevMs` | Processing delay |
| Calibration Error | 0.35px ± 0.10px | `cameraCalibError/Stddev` | Lens distortion error |

These can be tuned by overriding builder defaults to match your actual cameras.

### AprilTag Field Layout

The simulation automatically loads the official FRC field layout:

```java
AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(
    AprilTagFields.kDefaultField
);
```

This includes all AprilTag positions for the current game season.

## Monitoring Simulation

### SmartDashboard Telemetry

Key telemetry keys to monitor during simulation:

#### Vision System
- `Vision/Status` - Overall system status
- `Vision/Simulation` - "Active" when simulation running
- `Vision/FrontCamera/Connected` - Camera connection status
- `Vision/FrontCamera/TargetCount` - Number of tags detected
- `Vision/FrontCamera/EstimateStatus` - "Accepted", "Rejected", or "No valid estimate"
- `Vision/FrontCamera/EstimateX` - Estimated X position
- `Vision/FrontCamera/EstimateY` - Estimated Y position
- `Vision/StdDev/XY` - Current confidence (lower = more trust)
- `Vision/TagCount` - Total tags used in estimate
- `Vision/AvgDistance` - Average distance to detected tags
- `Vision/AcceptedPose/X` - Last accepted vision pose

#### Drivetrain
- `DriveTrain/field` - Field2d visualization (most important!)
- `DriveTrain/Pose/X` - Robot X position
- `DriveTrain/Pose/Y` - Robot Y position
- `DriveTrain/Pose/Rotation` - Robot heading

### Field2d Visualization

The Field2d widget shows:
- **Robot pose** (blue outline)
- **AprilTag locations** (if wireframe enabled)
- **Autonomous path** (if running auto)

**To view**:
1. Open Shuffleboard or Glass
2. Add "Field2d" widget
3. Select `DriveTrain/field` data source

## Vision Pose Estimation

### How It Works

The vision system uses PhotonVision's `PhotonPoseEstimator` to calculate robot pose from AprilTag detections:

1. **Tag Detection**: Camera detects one or more AprilTags
2. **Pose Calculation**: PhotonPoseEstimator uses tag positions to triangulate robot pose
3. **Quality Assessment**: System evaluates estimate quality based on:
   - Number of tags detected (more = better)
   - Distance to tags (closer = better)
   - Pose ambiguity (lower = better)
4. **Rejection Logic**: Poor estimates are filtered out
5. **Dynamic Weighting**: Confidence calculated based on conditions
6. **Fusion**: Vision measurement merged with odometry in `SwerveDrivePoseEstimator`

### Pose Estimation Strategies

Configured in `VisionSubsystemContext`:

- **MULTI_TAG_PNP_ON_COPROCESSOR** (default): Uses all visible tags, best accuracy
- **CLOSEST_TO_REFERENCE_POSE**: Selects estimate closest to current odometry
- **LOWEST_AMBIGUITY**: Uses single tag with lowest position uncertainty
- **AVERAGE_BEST_TARGETS**: Averages estimates from all tags

### Vision Confidence (Standard Deviations)

The system dynamically adjusts trust in vision measurements:

**High Trust** (low std dev):
- Multiple tags visible
- Tags close to robot (< 2 meters)
- Low ambiguity

**Low Trust** (high std dev):
- Single tag only
- Tags far from robot (> 4 meters)
- High ambiguity (> 0.2)

**Configuration** in `VisionSubsystemContext` (builder defaults):
```java
singleTagStdDevFactor = 4.0    // Start with low trust
multiTagStdDevFactor = 0.5     // Start with high trust
distanceScalingFactor = 0.1    // Increase uncertainty with distance
```

### Rejection Criteria

Vision estimates are rejected if:
- **Distance** > 4.0 meters from any tag
- **Ambiguity** > 0.2 (for single-tag estimates)

Adjust thresholds in `VisionSubsystemContext` builder:
```java
VisionSubsystemContext.builder()
    .maxPoseEstimationDistance(5.0)  // Allow 5m instead of 4m
    .poseAmbiguityThreshold(0.3)     // More lenient ambiguity
    .build();
```

## Testing Simulation

### Manual Testing

1. **Start simulation** and enable teleop mode
2. **Drive the robot** using gamepad
3. **Observe Field2d**: Robot should move smoothly
4. **Drive near AprilTags**: Vision should detect them
5. **Check telemetry**: Vision estimates should appear
6. **Watch pose corrections**: Robot pose should snap to tag positions when detected

### Expected Behavior

✅ **Good simulation**:
- Robot responds to joystick inputs
- Field2d shows smooth motion
- Vision detects tags within FOV
- Pose estimates accepted when conditions good
- Std dev decreases with multiple tags
- Robot pose corrects based on vision

❌ **Issues to investigate**:
- Robot doesn't move → Check motor simulation
- No tag detections → Check camera transforms
- All estimates rejected → Review rejection thresholds
- Pose jumps erratically → Tune std dev factors

## Simulation Limitations

### What Simulation Can't Do

- **Physical interactions**: No collision detection with field elements
- **Electrical issues**: No brownouts, CAN errors, or EMI
- **Mechanical wear**: No backlash, friction, or damage
- **Sensor noise**: Simplified noise models
- **Real network latency**: NetworkTables latency may differ

### Differences from Real Robot

| Aspect | Simulation | Real Robot |
|--------|------------|-----------|
| Physics | Idealized (no friction) | Real-world forces |
| Sensors | Perfect (unless configured) | Noise and drift |
| Timing | Deterministic | Variable loop times |
| Vision | Perfect detections | Lighting, motion blur |
| Network | Instant | Latency and packet loss |

## Calibration for Real Robot

When moving from simulation to real hardware:

### 1. Camera Transforms

Measure camera positions from CAD or physical robot and override in `VisionSubsystemContext`:

```java
// Example: Camera 0.5m forward, 0.3m up, level with robot
VisionSubsystemContext.builder()
    .frontCameraToRobot(new Transform3d(
        new Translation3d(0.5, 0.0, 0.3),
        new Rotation3d(0, 0, 0)
    ))
    .build();
```

**Critical**: Accurate transforms are essential for vision pose estimation.

### 2. Camera Properties

Match simulation to real cameras:
- Measure actual FOV
- Test detection distance
- Benchmark FPS and latency
- Calibrate using PhotonVision calibration tool

### 3. Vision Confidence Tuning

Test on real field and adjust in `VisionSubsystemContext`:
```java
VisionSubsystemContext.builder()
    .singleTagStdDevFactor(4.0)  // Increase if vision too trusted
    .multiTagStdDevFactor(0.5)   // Decrease if vision not trusted enough
    .build();
```

Watch for:
- Pose oscillation → Reduce trust (increase std dev)
- Vision ignored → Increase trust (decrease std dev)

### 4. Rejection Thresholds

Tune based on field testing in `VisionSubsystemContext`:
```java
VisionSubsystemContext.builder()
    .maxPoseEstimationDistance(4.0)  // Maximum reliable detection distance
    .poseAmbiguityThreshold(0.2)     // Maximum acceptable ambiguity
    .build();
```

## Troubleshooting

### Simulation Won't Start

- **Check Java version**: Must be Java 17+
- **Rebuild project**: `./gradlew clean build`
- **Check WPILib version**: Should be 2026.1.1
- **Look for build errors**: Fix compilation issues first

### No Vision Detections in Sim

- **Verify simulation enabled**: Check `VisionSubsystemContext.defaults()`
- **Check camera transforms**: Must not be all zeros (see Constants.Vision)
- **Drive near tags**: Robot must be within 4m and FOV
- **Check Field2d**: Verify robot position is reasonable

### Vision Estimates All Rejected

- **Too far from tags**: Drive closer (< 4m)
- **High ambiguity**: Move to see multiple tags
- **Strict thresholds**: Temporarily increase MAX_POSE_ESTIMATION_DISTANCE
- **Check telemetry**: Look at `Vision/AvgDistance` and rejection reason

### Pose Jumps Wildly

- **Std dev too low**: Vision trusted too much
- **Increase factors**: Try doubling MULTI_TAG_STD_DEV_FACTOR
- **Check transforms**: Incorrect camera position causes bad estimates
- **Multiple tag views**: Ensure robot sees consistent tags

## Advanced Topics

### Custom Camera Configurations

Add more cameras in `VisionSubsystem`:

```java
private final PhotonCamera thirdCamera = new PhotonCamera("camera-3");
private final PhotonPoseEstimator thirdPoseEstimator = new PhotonPoseEstimator(
    fieldLayout,
    context.getPoseEstimationStrategy(),
    customTransform
);
```

### Changing Pose Strategies

Modify strategy in `VisionSubsystemContext.defaults()`:

```java
.poseEstimationStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE)
```

Try different strategies to find best performance for your use case.

### Visualizing Vision in Sim

Enable wireframe rendering in `VisionSubsystem.initializeSimulation()`:

```java
frontCameraSim.enableDrawWireframe(true);  // Already enabled
```

This shows AprilTag outlines in Field2d when detected.

### Accessing Vision Debug Field

The VisionSystemSim has its own debug field:

```java
Field2d visionDebugField = visionSim.getDebugField();
SmartDashboard.putData("Vision/DebugField", visionDebugField);
```

Shows vision-specific visualization separate from drivetrain.

## Best Practices

### Development Workflow

1. **Write code** in VS Code
2. **Test in simulation** immediately
3. **Fix issues** found in sim
4. **Iterate** rapidly without hardware
5. **Deploy to robot** when confident
6. **Tune parameters** on real hardware
7. **Update simulation** to match reality

### Simulation Testing Checklist

Before deploying to real robot:

- [ ] Simulation starts without errors
- [ ] Robot responds to all commands
- [ ] Autonomous routines complete successfully
- [ ] Vision detects tags in expected scenarios
- [ ] Pose estimation performs well
- [ ] All telemetry values are reasonable
- [ ] No exceptions or crashes occur

### Parameter Tuning

When tuning vision parameters:

1. **Start conservative**: High std devs (low trust)
2. **Test thoroughly**: Many scenarios and tag configurations
3. **Tune gradually**: Small adjustments at a time
4. **Document changes**: Note why parameters were changed
5. **Validate on field**: Always test on official field layout

## Resources

### Documentation

- [WPILib Simulation Guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
- [PhotonVision Docs](https://docs.photonvision.org/)
- [PhotonVision Simulation](https://docs.photonvision.org/en/latest/docs/simulation/simulation-java.html)
- [PhotonVision Pose Estimation Example](https://docs.photonvision.org/en/latest/docs/examples/poseest.html)

### Code References

- `Drivetrain.simulationPeriodic()` - Drivetrain physics simulation
- `VisionSubsystem.initializeSimulation()` - Vision simulation setup
- `VisionSubsystem.updatePoseEstimation()` - Pose estimation logic
- `VisionSubsystem.calculateVisionStdDevs()` - Dynamic confidence calculation
- `VisionSubsystemContext` - All vision configuration parameters with builder defaults

## Contributing

When adding new simulation features:

1. Follow the existing real/sim conditional pattern
2. Add telemetry for debugging
3. Document behavior in this file
4. Test thoroughly in simulation before hardware
5. Provide calibration instructions for real robot

---

**Questions or issues?** Check GitHub issues or ask the software lead.

**Last Updated**: January 2026
