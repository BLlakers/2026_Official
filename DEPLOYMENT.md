# Deployment Guide: Test Bot & PhotonVision Camera Calibration

**Team 2534 BL Lakers | 2026 Season**

This guide walks through deploying the codebase to the test bot for the first time, with a focus on positioning and calibrating PhotonVision cameras.

---

## Table of Contents

- [Prerequisites](#prerequisites)
- [Network & Hardware Setup](#network--hardware-setup)
- [Building & Deploying](#building--deploying)
- [First Boot Checklist](#first-boot-checklist)
- [PhotonVision Camera Setup](#photonvision-camera-setup)
- [Camera Calibration Workflow](#camera-calibration-workflow)
- [Updating Camera Transforms in Code](#updating-camera-transforms-in-code)
- [Validating Vision Pose Estimation](#validating-vision-pose-estimation)
- [Monitoring & Telemetry](#monitoring--telemetry)
- [Troubleshooting](#troubleshooting)
- [Code Changes Checklist](#code-changes-checklist)

---

## Prerequisites

Before arriving at the test bot:

### On Your Laptop

| Tool | Purpose | Install |
|------|---------|---------|
| **WPILib 2026** | Build toolchain + Driver Station | [wpilib.org](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) |
| **FRC Driver Station** | Enable/disable robot | Comes with WPILib (Windows only) |
| **FRC Game Tools** | roboRIO imaging, DS | [NI FRC Game Tools](https://www.ni.com/en/support/downloads/drivers/download.frc-game-tools.html) |
| **PhotonVision Client** | Camera configuration UI | Access via browser at `http://photonvision.local:5800` |
| **Shuffleboard or SmartDashboard** | Live telemetry | Comes with WPILib |
| **AdvantageScope** | Log replay & field visualization | [github.com/Mechanical-Advantage/AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope) |
| **JDK 17** | Build the code | Comes with WPILib |

### On the Robot

| Component | Details |
|-----------|---------|
| **roboRIO 2** | Imaged with 2026 firmware |
| **REV Power Distribution Hub** | CAN ID 20 |
| **NavX-MXP** | Plugged into MXP SPI port |
| **4x Swerve Modules** | REV SPARK MAX NEOs (CAN IDs: 1-8) |
| **3x PhotonVision Cameras** | USB or Ethernet to coprocessor(s) |
| **PhotonVision Coprocessor(s)** | Raspberry Pi / Orange Pi with PhotonVision installed |
| **Radio** | Programmed for team 2534 |

### Printed Materials

- Several **2026 FRC AprilTags** (36h11 family, printed at correct size - 6.5" for 2026)
- A **tape measure** and **angle finder/protractor** for measuring camera positions
- **Masking tape** to mark positions on the floor

---

## Network & Hardware Setup

### Robot Network

When connected to the robot's radio, the network follows the FRC convention:

| Device | Hostname / IP |
|--------|--------------|
| roboRIO | `roborio-2534-frc.local` or `10.25.34.2` |
| Driver Station laptop | `10.25.34.5` (or DHCP) |
| PhotonVision coprocessor(s) | `10.25.34.11`, `.12`, `.13` (static recommended) |

### Connecting Your Laptop

1. Connect to the robot's radio WiFi or use a direct Ethernet cable to the radio
2. Verify connectivity: `ping roborio-2534-frc.local`
3. If using mDNS fails, use the IP directly: `ping 10.25.34.2`

### CAN Bus Wiring Reference

The deployed code expects these CAN IDs. Verify before deploying:

```
Swerve Drivetrain (SPARK MAX / NEO):
  Front Left:   Drive=3, Steer=4
  Front Right:  Drive=6, Steer=5
  Rear Left:    Drive=2, Steer=1
  Rear Right:   Drive=7, Steer=8

Turn Encoders (CTRE Mag Encoder, PWM on DIO):
  DIO 0: Rear Left
  DIO 1: Front Left
  DIO 2: Front Right
  DIO 3: Rear Right

Other:
  PDH: CAN 20
  Feeder Motor: CAN 13
  Intake/Launcher Motor: CAN 14
```

> **TEST BOT WARNING:** If your test bot has different CAN IDs than above, you will need to update `Constants.java` before deploying. See [Code Changes Checklist](#code-changes-checklist).

---

## Building & Deploying

### Step 1: Build Locally First

From the project root (`2026_Official/`):

```bash
# Windows
.\gradlew.bat build

# macOS/Linux
./gradlew build
```

This compiles the code, runs tests, and checks formatting. Fix any errors before deploying.

### Step 2: Deploy to the roboRIO

Make sure your laptop is on the robot's network, then:

```bash
# Windows
.\gradlew.bat deploy

# macOS/Linux
./gradlew deploy
```

**What happens:**
1. Code compiles into a fat JAR
2. Static files from `src/main/deploy/` are copied to `/home/lvuser/deploy/` on the roboRIO
3. The JAR is uploaded to the roboRIO
4. Robot code restarts automatically

### Step 3: Verify Deployment

1. Open **FRC Driver Station** on your laptop
2. The **Communications** light should go green (connected to roboRIO)
3. The **Robot Code** light should go green (code is running)
4. The robot should be in **Disabled** mode

If the Robot Code light stays red:
- Check Driver Station console (bottom pane) for Java exceptions
- SSH into the roboRIO and check logs: `ssh admin@10.25.34.2` then `cat /home/lvuser/FRC_UserProgram.log`

---

## First Boot Checklist

With the robot on blocks (wheels off the ground) and code deployed:

### 1. Driver Station Verification

- [ ] Communications light is green
- [ ] Robot Code light is green
- [ ] No errors in the DS console

### 2. Open Shuffleboard / SmartDashboard

Connect to NetworkTables at `roborio-2534-frc.local` or `10.25.34.2`.

- [ ] `Telemetry/Level` shows `LAB`
- [ ] `Telemetry/Initialized` shows `true`
- [ ] `Code Version` appears

### 3. Gamepad Setup (Driver Station)

Plug in controllers and assign:

```
USB 0: Driver Controller (swerve drive)
USB 1: Manipulator Controller (fuel subsystem)
USB 2: Debug Controller (reserved)
```

### 4. Basic Drive Test (on blocks)

1. Enable **TeleOp**
2. Push driver controller left stick forward - wheels should spin forward
3. Push right stick - robot should attempt to rotate
4. Press **B** on driver controller to reset the gyro
5. Press **right stick button** to toggle wheel lock (X-pattern)

### 5. Swerve Module Alignment Check

If wheels point the wrong direction at startup:
- The turn encoder offsets need recalibration for the test bot
- See [Code Changes Checklist](#code-changes-checklist) for how to measure new offsets

### 6. Vision System Status

Before cameras are set up, you should see:
- `Vision/Status` = `"No Cameras Connected"`
- `Vision/FrontRightCamera/Connected` = `false`
- `Vision/FrontLeftCamera/Connected` = `false`
- `Vision/RearCamera/Connected` = `false`

This is expected. Continue to the next section to set up the cameras.

---

## PhotonVision Camera Setup

### Physical Mounting

The code expects **3 cameras** mounted as follows (these are defaults - you will measure and update the real values):

| Camera | Default Position | Default Angle |
|--------|-----------------|---------------|
| **Front-Right** | 30cm forward, 25cm right, 25cm up | 15 deg down, 30 deg right |
| **Front-Left** | 30cm forward, 25cm left, 25cm up | 15 deg down, 30 deg left |
| **Rear** | 30cm backward, centered, 25cm up | 15 deg down, facing backward |

> **For initial testing**, even **one camera** is enough. You can start with just the front-right camera and add the others later.

### Coprocessor Setup

Each camera connects to a coprocessor running PhotonVision. For each coprocessor:

1. **Flash PhotonVision** onto the coprocessor if not already done
   - Download from [photonvision.org](https://photonvision.org)
   - Follow the official flashing guide for your hardware (Raspberry Pi, Orange Pi, etc.)

2. **Connect the camera** via USB to the coprocessor

3. **Connect the coprocessor** to the robot network (Ethernet to the radio/switch)

4. **Set a static IP** (recommended):
   - Access PhotonVision UI at `http://photonvision.local:5800`
   - Go to **Settings** > **Networking**
   - Set a static IP in the `10.25.34.x` range:
     - Front-Right coprocessor: `10.25.34.11`
     - Front-Left coprocessor: `10.25.34.12`
     - Rear coprocessor: `10.25.34.13`
   - Set the **NetworkTables server** to `10.25.34.2` (roboRIO IP)

### PhotonVision Pipeline Configuration

For each camera in the PhotonVision UI:

1. **Name the camera** (Settings > Camera tab) to match the code:
   - `photonvision-front-right`
   - `photonvision-front-left`
   - `photonvision-rear`

   **CRITICAL:** These names must match **exactly** (case-sensitive). If they don't match, the robot code won't find the camera in NetworkTables.

2. **Create an AprilTag Pipeline:**
   - Click **+ New Pipeline**
   - Pipeline type: **AprilTag**
   - Tag family: **36h11**

3. **Enable multi-target mode:**
   - In the pipeline settings, enable **Multi-Target** (sometimes called 3D or SolvePNP)
   - This allows the coprocessor to compute pose from multiple tags simultaneously

4. **Tune exposure and brightness:**
   - Adjust until AprilTags are reliably detected
   - Lower exposure = sharper tags but darker image
   - Higher exposure = brighter but more motion blur
   - Aim for consistent detection at 1-4 meter range

5. **Camera calibration (intrinsics):**
   - Go to **Cameras** > **Camera Calibration**
   - Use a checkerboard pattern (print one from the PhotonVision docs)
   - Take 12-15 images from different angles
   - Run calibration - this computes lens distortion correction
   - **This is essential** for accurate pose estimation

### Verify Camera Connection from Robot Code

After setting up at least one camera:

1. Deploy robot code (if not already): `.\gradlew.bat deploy`
2. Open Shuffleboard
3. Check:
   - `Vision/FrontRightCamera/Connected` should be `true`
   - `Vision/Status` should change from "No Cameras Connected" to something else

If a camera shows `false`:
- Verify the camera name matches exactly in PhotonVision UI
- Check that the coprocessor can reach the roboRIO (ping test)
- Restart PhotonVision service on the coprocessor
- Check the DS console for NetworkTables connection errors

---

## Camera Calibration Workflow

This is the primary goal of this deployment: physically position cameras and measure their transforms so the code can accurately estimate the robot's position on the field.

### What You Need

- Tape measure (metric, in meters/centimeters)
- Angle finder or digital inclinometer (for pitch angle)
- Protractor or angle measurement tool (for yaw angle)
- Notepad to record measurements
- Several printed AprilTags placed at known positions

### Step 1: Mount Cameras and Measure Transforms

For **each camera**, measure these values from the **robot center** (center of rotation, at floor level):

1. **X (forward/backward):** Distance forward from robot center in meters
   - Positive = forward of center
   - Negative = behind center

2. **Y (left/right):** Distance left/right from robot center in meters
   - Positive = left of center
   - Negative = right of center

3. **Z (height):** Distance above the floor in meters
   - Always positive

4. **Pitch (tilt):** Camera tilt angle in degrees
   - 0 = looking straight ahead
   - Negative = tilted down (most cameras should be tilted 10-20 deg down)

5. **Yaw (rotation):** Camera horizontal angle in degrees
   - 0 = pointing straight forward
   - Positive = angled left
   - Negative = angled right
   - 180 = pointing straight backward

Record your measurements:

```
Front-Right Camera:
  X = ______ m    Y = ______ m    Z = ______ m
  Pitch = ______ deg    Yaw = ______ deg

Front-Left Camera:
  X = ______ m    Y = ______ m    Z = ______ m
  Pitch = ______ deg    Yaw = ______ deg

Rear Camera:
  X = ______ m    Y = ______ m    Z = ______ m
  Pitch = ______ deg    Yaw = ______ deg
```

### Step 2: Place AprilTags at Known Positions

For calibration validation, set up a mini test field:

1. Pick a clear area of floor (at least 4m x 4m)
2. Mark an **origin point** with tape
3. Place 2-3 printed AprilTags at **measured distances** from the origin
4. Use tag IDs that exist in the 2026 field layout (e.g., tags 1-28)
5. Mount tags vertically (facing the robot) at roughly 1.1m height (hub tag height)
6. Record exact positions of each tag

### Step 3: Test Detection at Various Distances

With the robot enabled in TeleOp and cameras connected:

1. Place robot at a known distance from a tag (start at 1 meter)
2. Check Shuffleboard:
   - `Vision/{Camera}/TargetCount` > 0
   - `Vision/{Camera}/DetectedTags` shows the correct tag ID
   - `Vision/{Camera}/BestTargetYaw` changes as you move the robot laterally
3. Move to 2m, 3m, 4m and note when detection becomes unreliable
4. Record the **maximum reliable detection distance** for each camera

### Step 4: Validate Pose Estimation

The vision system already feeds pose estimates into the drivetrain's SwerveDrivePoseEstimator. To validate:

1. Place the robot at a known position relative to visible AprilTags
2. Open Shuffleboard and watch:
   - `Vision/{Camera}/EstimateStatus` should show "Accepted"
   - `Vision/{Camera}/EstimateX` and `EstimateY` should roughly match the known position
3. If estimates show "Rejected":
   - Single tags beyond 4m are rejected (distance filter)
   - High ambiguity tags are rejected (> 0.2 threshold)
   - This is working as intended
4. Drive the robot slowly and confirm the pose estimate tracks smoothly
5. Compare `CurrentPoseEstimator` (vision-fused) vs `CurrentPose` (odometry-only) in Shuffleboard

### Step 5: Iterate on Camera Positions

Based on testing:
- Adjust camera tilt (pitch) if tags aren't detected at desired ranges
- Adjust camera yaw if the field of view doesn't cover the right area
- Ensure no part of the robot frame blocks the camera view
- After each physical adjustment, **re-measure and update the transforms**

---

## Updating Camera Transforms in Code

Once you have final measurements, update `VisionSubsystemContext.java`:

### File: `src/main/java/frc/robot/subsystems/vision/VisionSubsystemContext.java`

The default transforms are at lines 54-77. Update them with your measured values:

```java
// Example: Front-Right camera measured at
// 28cm forward, 22cm right, 30cm up, tilted 12deg down, angled 25deg right
@Builder.Default
private final Transform3d frontRightCameraToRobot = new Transform3d(
        new Translation3d(0.28, -0.22, 0.30),
        new Rotation3d(0, Math.toRadians(-12), Math.toRadians(-25)));

// Example: Front-Left camera measured at
// 28cm forward, 22cm left, 30cm up, tilted 12deg down, angled 25deg left
@Builder.Default
private final Transform3d frontLeftCameraToRobot = new Transform3d(
        new Translation3d(0.28, 0.22, 0.30),
        new Rotation3d(0, Math.toRadians(-12), Math.toRadians(25)));

// Example: Rear camera measured at
// 25cm backward, centered, 28cm up, tilted 10deg down, facing backward
@Builder.Default
private final Transform3d rearCameraToRobot = new Transform3d(
        new Translation3d(-0.25, 0.0, 0.28),
        new Rotation3d(0, Math.toRadians(-10), Math.PI));
```

After updating, redeploy: `.\gradlew.bat deploy`

---

## Validating Vision Pose Estimation

### Using AdvantageScope

AdvantageScope gives you the best visualization of whether your camera transforms are correct.

1. **Plug a USB stick** (FAT32 formatted) into the roboRIO USB port
2. Run the robot for a few minutes, driving near AprilTags
3. Retrieve the USB stick - logs are in `FRC_LOGS/` folder
4. Open the `.wpilog` file in AdvantageScope

In AdvantageScope:
- Add `Drivetrain/Pose` to the **Field 2D** view (odometry-only pose)
- Add `CurrentPoseEstimator` (vision-fused pose)
- If the vision-fused pose jumps wildly, the camera transforms may be wrong
- If the vision-fused pose is smooth and corrects drift, the calibration is good

### Using Shuffleboard Live

Key values to monitor during a calibration run:

| NetworkTables Key | What to Look For |
|-------------------|------------------|
| `Vision/Status` | "Tracking (FR+FL+Rear)" when all cameras see tags |
| `Vision/TotalTagsDetected` | Number of tags currently visible |
| `Vision/FrontRightCamera/EstimateStatus` | "Accepted" means pose was used |
| `Vision/StdDev/XY` | Lower = more trusted. Multi-tag should be ~0.5-1.0 |
| `Vision/TagCount` | 2+ tags = multi-tag mode (more accurate) |
| `Vision/AvgDistance` | Distance to detected tags in meters |
| `CurrentPoseEstimator` | Robot pose with vision corrections applied |

### Signs of Good Calibration

- Vision-fused pose (`CurrentPoseEstimator`) is stable and doesn't jump
- Pose corrections are small and smooth when tags become visible
- Multi-tag estimates (2+ tags) produce especially stable poses
- Robot pose on the AdvantageScope field matches physical position

### Signs of Bad Calibration

- Pose jumps wildly when vision estimates are accepted
- Robot appears to be in the wrong location on the field visualization
- `EstimateStatus` frequently shows "Rejected"
- Large discrepancy between odometry-only and vision-fused poses

---

## Monitoring & Telemetry

### Telemetry Level

The deployed code defaults to `LAB` level, which captures detailed data useful for testing. This is configured in `src/main/deploy/telemetry.properties`:

```properties
telemetry.level=LAB
```

You can change the level live via Shuffleboard by editing `Telemetry/Level`.

### Key Telemetry to Watch

#### Drivetrain Health
- `Drivetrain/Pose` - Odometry-only robot position
- `CurrentPoseEstimator` - Vision-fused position
- `Drivetrain/Heading` - Current gyro heading

#### Vision Health
- `Vision/Status` - Quick overview of camera status
- `Vision/{Camera}/Connected` - Per-camera connection status
- `Vision/{Camera}/TargetCount` - How many tags each camera sees
- `Vision/{Camera}/DetectedTags` - Which tag IDs are visible

#### Turret Tracker (for hub targeting)
- `TurretTracker/AngleDeg` - Computed aim angle
- `TurretTracker/DistanceM` - Distance to target hub
- `TurretTracker/Mode` - SHOOTING or PASSING mode

### Log Collection

For post-session analysis:

1. **USB stick** (recommended): Insert FAT32 USB into roboRIO before testing
   - Logs auto-save to `FRC_LOGS/` on the stick
   - File format: `FRC_YYYYMMDD_HHMMSS.wpilog`
2. **Internal storage** (fallback): Logs go to `/home/lvuser/logs/`
   - SSH in to retrieve: `scp admin@10.25.34.2:/home/lvuser/logs/*.wpilog .`

Open logs in **AdvantageScope** for full replay with field visualization.

---

## Troubleshooting

### Robot Code Won't Start

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Robot Code light stays red | Java exception on startup | Check DS console for stack trace |
| CAN errors in DS console | Wrong CAN IDs for test bot | Update `Constants.java` Port class |
| "HAL: Resource already allocated" | DIO channel conflict | Check encoder DIO assignments |

### Cameras Not Detected

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| `Connected = false` for all cameras | Coprocessor not on network | Check Ethernet, ping coprocessor IP |
| `Connected = false` for one camera | Camera name mismatch | Check PhotonVision UI > camera name matches exactly |
| Camera connected but no targets | Pipeline not active | Check PhotonVision UI > pipeline is running (green) |
| Intermittent detection | Poor lighting or focus | Adjust exposure in PhotonVision UI |

### Swerve Drive Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Wheels point wrong direction | Turn encoder offsets wrong for test bot | Re-measure offsets (see code changes) |
| Robot drives sideways | Module assignment mismatch | Verify CAN IDs match physical positions |
| Robot spins in circles | Gyro not calibrated | Wait 10 sec after boot, press B to reset |
| Very slow or no movement | Right trigger not pressed | Right trigger = gas pedal |

### Vision Pose Estimation Issues

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Pose jumps wildly | Camera transform is wrong | Re-measure physical camera positions |
| All estimates rejected | Tags too far or too ambiguous | Move closer, add more tags |
| Estimates only from one camera | Other cameras misconfigured | Check each camera individually |

---

## Code Changes Checklist

These are changes you **may** need to make before or during the test bot deployment.

### Required If Test Bot Has Different CAN IDs

Update motor CAN IDs in `Constants.java` (`Port` class, lines 107-130):

```java
public static class Port {
    public static final int REAR_LEFT_TURN_CHANNEL = 1;      // Update if different
    public static final int REAR_LEFT_DRIVE_CHANNEL = 2;      // Update if different
    public static final int FRONT_LEFT_DRIVE_CHANNEL = 3;     // Update if different
    // ... etc
}
```

### Required: Measure Turn Encoder Offsets for Test Bot

The current offsets are for the 2025 robot. The test bot will have different values.

**How to measure offsets:**

1. Deploy the code with all offsets set to `0`:
   ```java
   // In Constants.java, temporarily:
   public class RobotVersion2025 extends RobotVersionConstants {
       public static final double flTurnEncoderOffset = 0;
       public static final double frTurnEncoderOffset = 0;
       public static final double rlTurnEncoderOffset = 0;
       public static final double rrTurnEncoderOffset = 0;
   }
   ```
2. Deploy and enable the robot
3. Physically straighten all wheels to point forward (use a straight edge)
4. Read the raw encoder values from Shuffleboard for each module
5. The offset = the raw value when wheels are straight forward
6. Update the constants and redeploy

### Required: Update Camera Transforms After Measurement

As described in [Updating Camera Transforms in Code](#updating-camera-transforms-in-code), update `VisionSubsystemContext.java` with measured camera positions.

### Optional: Starting With Fewer Than 3 Cameras

If you only have 1 or 2 cameras to start with, the code handles this gracefully - unconnected cameras simply show `Connected = false` and no data. The pose estimator only uses cameras that provide valid data.

No code changes needed - just physically connect however many cameras you have.

### Optional: Adjusting Vision Tuning Parameters

If vision estimates are too noisy or too conservative, adjust in `VisionSubsystemContext.java`:

```java
// Reject estimates from tags further than this (meters)
private final double maxPoseEstimationDistance = 4.0;   // Increase for larger test area

// Reject single-tag estimates with ambiguity above this
private final double poseAmbiguityThreshold = 0.2;      // Increase to accept more, decrease for stricter

// Trust level: lower = more trusted
private final double singleTagStdDevFactor = 4.0;       // Single tag trust (higher = less trust)
private final double multiTagStdDevFactor = 0.5;         // Multi tag trust (lower = more trust)
```

### Optional: Telemetry Level for Verbose Debugging

If you need maximum diagnostic data during calibration, edit `src/main/deploy/telemetry.properties`:

```properties
telemetry.level=VERBOSE
```

This logs per-tag yaw/pitch/area/skew data for every detected AprilTag. Useful during calibration, but generates large log files.

---

## Quick Reference: Deploy Cycle

```
1. Make code changes
2. Build:     .\gradlew.bat build
3. Deploy:    .\gradlew.bat deploy
4. Open Driver Station
5. Open Shuffleboard (connect to 10.25.34.2)
6. Enable robot (TeleOp for driving, Test for diagnostics)
7. Monitor telemetry
8. Disable robot
9. Retrieve USB logs if needed
10. Repeat
```

## Quick Reference: Camera Names

| Code Name | PhotonVision NetworkTables Name |
|-----------|-------------------------------|
| Front-Right Camera | `photonvision-front-right` |
| Front-Left Camera | `photonvision-front-left` |
| Rear Camera | `photonvision-rear` |

## Quick Reference: Controller Layout

```
DRIVER (USB 0):
  Left Stick  = Drive (translation)
  Right Stick = Rotate
  Right Trigger = Gas (acceleration)
  Left Trigger  = Half-speed (hold > 50%)
  B Button    = Reset gyro
  R-Stick Click = Wheel lock toggle

MANIPULATOR (USB 1):
  Left Bumper  = Fuel intake (hold)
  Right Bumper = Fuel launch (hold: 1s spinup then launch)
  X Button     = Fuel eject (hold)
```
