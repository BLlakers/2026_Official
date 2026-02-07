# Turret Hub Tracking & Vision System

Design document for the turret tracker (`frc.robot.subsystems.turrettracker`) and the
triple-camera vision system (`frc.robot.subsystems.vision`).

---

## Table of Contents

1. [What This Does](#1-what-this-does)
2. [High-Level Design](#2-high-level-design)
3. [The Hub Model](#3-the-hub-model)
4. [The Math](#4-the-math)
5. [Interactive Math Visualizations (Desmos)](#5-interactive-math-visualizations-desmos)
6. [WPILib Visualization APIs](#6-wpilib-visualization-apis)
7. [NetworkTables and AdvantageScope](#7-networktables-and-advantagescope)
8. [Triple-Camera Vision System](#8-triple-camera-vision-system)
9. [File-by-File Implementation Notes](#9-file-by-file-implementation-notes)
10. [How to View It In Sim](#10-how-to-view-it-in-sim)
11. [Configuration and Tuning](#11-configuration-and-tuning)
12. [Next Steps](#12-next-steps)

---

## 1. What This Does

The system has two main components:

### Turret Tracker

The `TurretTracker` is a **software-only subsystem** with no physical motor. Every 20ms
(the robot's periodic loop rate), it answers one question:

> "If I had a turret on this robot, what angle would it need to be at to aim at the hub?"

It reads the robot's current pose from the drivetrain's pose estimator, computes the
geometric center of the alliance hub from AprilTag positions, calculates the angle to
aim at that center point, clamps it to the turret's range of motion (270 degrees), and
publishes the result so you can see it working in simulation.

The hub is a top-entry target (like a basketball hoop with an intake scoop on top).
The ball enters from above, so the turret always aims at the **hub center** regardless
of which side the robot is on. There is no need for face selection.

There is no PID controller, no motor output, no closed-loop control. This is the
**geometric targeting layer** that a future turret motor controller would consume.

### Triple-Camera Vision System

The `VisionSubsystem` manages three PhotonVision cameras for AprilTag-based localization:

- **Front-right** camera: angled 30 degrees outward to the right
- **Front-left** camera: angled 30 degrees outward to the left
- **Rear** camera: facing backward

Together, the two front cameras provide approximately 150 degrees of forward coverage
with a 60-degree overlap zone in the center. The rear camera covers 90 degrees behind
the robot. Combined, the system provides near-360-degree AprilTag detection.

Each camera feeds into its own `PhotonPoseEstimator`, and all three contribute vision
measurements to the drivetrain's `SwerveDrivePoseEstimator` for sensor-fused localization.

In simulation, FOV (field-of-view) cone visualizations are published to AdvantageScope
so you can see exactly what each camera can see.

---

## 2. High-Level Design

### Turret Tracker Data Flow

```
                                  +-------------------+
                                  | AprilTagField-    |
                                  | Layout (WPILib)   |
                                  | Tag positions for |
                                  | the 2026 field    |
                                  +--------+----------+
                                           |
                                           | getTagPose(id) at construction
                                           v
+---------------+    getPose2d    +-------------------+
|  Drivetrain   | -------------> |   TurretTracker   |
| (pose         |   Estimator()  |                   |
|  estimator)   |                | 1. Get hub center  |
+---------------+                | 2. Compute angle  |
                                 | 3. Clamp to range |
                                 | 4. Publish viz    |
       +-------------+          +----+---------+-----+
       | DriverStation|               |         |
       | .getAlliance()|              |         |
       +------+-------+              |         |
              |                       v         v
              |              Mechanism2d   StructPublisher
              +-- selects -->  (SimGUI)    (AdvantageScope)
                 blue/red hub
```

### Vision System Data Flow

```
+---------------------+     +---------------------+     +---------------------+
| PhotonCamera        |     | PhotonCamera        |     | PhotonCamera        |
| "front-right"       |     | "front-left"        |     | "rear"              |
| yaw=-30deg          |     | yaw=+30deg          |     | yaw=180deg          |
+----------+----------+     +----------+----------+     +----------+----------+
           |                           |                           |
           v                           v                           v
+----------+----------+     +----------+----------+     +----------+----------+
| PhotonPoseEstimator |     | PhotonPoseEstimator |     | PhotonPoseEstimator |
| MULTI_TAG_PNP       |     | MULTI_TAG_PNP       |     | MULTI_TAG_PNP       |
+----------+----------+     +----------+----------+     +----------+----------+
           |                           |                           |
           | reject / accept           | reject / accept           | reject / accept
           | + dynamic std devs        | + dynamic std devs        | + dynamic std devs
           |                           |                           |
           +---------------------------+---------------------------+
                                       |
                                       v
                          +----------------------------+
                          | Drivetrain                 |
                          | SwerveDrivePoseEstimator   |
                          | addVisionMeasurement()     |
                          +----------------------------+
```

### Turret Tracker Data Flow Steps

1. **Input**: Robot pose (from `Drivetrain.getPose2dEstimator()`) and alliance color
   (from `DriverStation.getAlliance()`).

2. **Hub center**: Computed once at construction time. The constructor reads every hub
   face tag's 3D pose from the field layout JSON, averages the midpoints of all 4 faces,
   and stores the result as a single `Translation2d` per hub.

3. **Angle computation**: Standard atan2 trigonometry to get the field-relative angle
   from robot to hub center, then subtract the robot's heading to get a robot-relative
   angle.

4. **Clamping**: If the robot-relative angle exceeds +/-135 degrees (half of the 270-degree
   range), the turret angle is clamped to the nearest limit and `targetInRange` is set
   to `false`.

5. **Output**: The computed angle, distance, and status are published via three channels:
   Mechanism2d widget, NetworkTables struct publishers, and the telemetry system.

---

## 3. The Hub Model

### Physical structure

Each hub in the 2026 game is a square-ish structure with 4 outward-facing walls (faces).
Each face has 2 AprilTags mounted on it at Z = 1.124m. There are also 4 corner tags at
Z = 0.889m which we ignore for aiming purposes.

### Tag assignments (from `2026-rebuilt-welded.json`)

**Blue Hub** (center approximately X=4.63, Y=4.03):

```
                    North face
                  Tags 21, 24
                  Y = 4.638
            +-------------------+
            |                   |
  West face |                   | East face
  Tags 25,26|       HUB         | Tags 19,20
  X = 4.022 |                   | X = 5.229
            |                   |
            +-------------------+
                  Tags 18, 27
                  Y = 3.431
                    South face
```

**Red Hub** (center approximately X=12.09, Y=4.03):

```
                    North face
                   Tags 2, 11
                   Y = 4.638
            +-------------------+
            |                   |
  West face |                   | East face
  Tags 3, 4 |       HUB         | Tags 9, 10
  X = 11.312|                   | X = 12.519
            |                   |
            +-------------------+
                   Tags 5, 8
                   Y = 3.431
                    South face
```

### How the hub center is computed

The `computeHubCenter` method averages all four face midpoints:

1. **Compute each face's midpoint**: Average the (X, Y) positions of the two tags on
   that face. For example, blue West face tags 25 and 26 are at (4.022, 4.390) and
   (4.022, 4.035), so the midpoint is (4.022, 4.212).

2. **Average all face midpoints**: The hub center is the mean of the four face midpoints.
   This gives the geometric center of the hub structure.

The turret always aims at this single center point. Since the hub is a top-entry target
(like a basketball hoop), the specific face the robot is near doesn't matter -- the ball
goes in from above.

---

## 4. The Math

### 4.1 Dot product (reference material)

> **Note**: The current implementation does NOT use dot product face selection. The turret
> always aims at the hub center because the hub is a top-entry target. This section is
> retained as reference material since the Desmos visualization covers it and it's a
> generally useful concept for FRC.

The dot product of two vectors A and B equals `|A| * |B| * cos(theta)` where theta is the
angle between them. When theta < 90 degrees, cos is positive (vectors point in roughly the
same direction). When theta > 90 degrees, cos is negative (they point apart).

This would be useful if the hub had side-entry openings and you needed to pick which face
to shoot into. In that case, you'd compute `dot = robotToFace . faceNormal` for each face
and only aim at faces where dot > 0 (robot is in front of the face).

### 4.2 Angle to hub center (atan2)

We need the angle from the robot to the hub center.

```java
double dx = hubCenter.getX() - robot.getX();
double dy = hubCenter.getY() - robot.getY();
double fieldAngleRad = Math.atan2(dy, dx);
```

`Math.atan2(y, x)` returns the angle in radians from the positive X-axis to the point
(x, y), measured counter-clockwise. The result is in the range [-pi, pi].

```
            +Y
            |
            |  atan2(1, 1) = pi/4 (45 degrees)
            |  /
            | /
  ----------+----------> +X
            |
            |
```

This gives us the **field-relative** angle -- the direction from robot to target in the
field's coordinate system.

### 4.3 Converting to robot-relative angle

The turret sits on the robot and rotates relative to the robot's body. If the robot is
facing east (heading = 0) and the target is to the north (field angle = 90 degrees), the
turret needs to point 90 degrees to the left. But if the robot is facing north (heading =
90 degrees) and the target is also to the north, the turret points straight ahead (0
degrees).

```java
double robotRelativeRad = fieldAngleRad - robotHeadingRad;
```

This subtraction converts from field-relative to robot-relative. But the result can end up
outside [-pi, pi], which would cause the turret to "wrap around" incorrectly.

### 4.4 Angle normalization

```java
robotRelativeRad = Math.atan2(Math.sin(robotRelativeRad), Math.cos(robotRelativeRad));
```

This is a standard trick to normalize any angle to [-pi, pi]. Here's why it works:

- `sin` and `cos` decompose the angle into its Y and X components on the unit circle.
- `atan2` reconstructs the angle from those components, always returning a value in
  [-pi, pi].
- If the input was, say, 270 degrees (3pi/2), sin = -1, cos = 0, and atan2(-1, 0) = -pi/2
  (-90 degrees). That's the same physical direction, just expressed in the canonical range.

**Why not use modulo?** The `%` operator in Java doesn't handle negative values the way
you'd want for angles. `atan2(sin, cos)` is cleaner and always correct.

### 4.5 Range clamping

```java
double halfRange = 270.0 / 2.0;  // 135 degrees
targetInRange = Math.abs(rawAngleDegrees) <= halfRange;

if (targetInRange) {
    turretAngleDegrees = rawAngleDegrees;
} else {
    turretAngleDegrees = Math.copySign(halfRange, rawAngleDegrees);
}
```

`Math.copySign(magnitude, sign)` returns `magnitude` with the sign of `sign`. So if the
raw angle is -150 degrees (target is behind-left), it clamps to -135 degrees (the turret's
left limit).

### 4.6 Converting back to field-relative for visualization

The Mechanism2d shows the turret in robot-space (which is what we want for the dial). But
for the AdvantageScope aim line on the field, we need to convert back:

```java
double aimFieldAngleRad = robotHeadingRad + turretAngleDegrees_in_radians;
```

This reverses the subtraction from step 4.3. The aim line endpoint is then:

```java
endX = robotX + length * cos(aimFieldAngleRad);
endY = robotY + length * sin(aimFieldAngleRad);
```

### 4.7 FOV cone projection (vision system)

Each camera's field-of-view is visualized as a V-shaped cone on the field. The math
projects the camera's position and FOV edges from robot-relative to field-relative
coordinates.

**Camera position in field coordinates:**

```java
// Rotate camera's robot-relative offset by the robot's heading
camX = robotX + cam.x * cos(heading) - cam.y * sin(heading)
camY = robotY + cam.x * sin(heading) + cam.y * cos(heading)
```

This is a standard 2D rotation matrix application. The camera's mounting position
(e.g., front-right at x=+0.30m, y=-0.25m) is rotated by the robot's heading to get
its field-absolute position.

**Camera heading in field coordinates:**

```java
camHeading = robotHeading + cameraYaw
```

Where `cameraYaw` is the camera's mounting angle (e.g., -30 degrees for the front-right
camera). This gives the direction the camera is pointing in field coordinates.

**FOV edge computation:**

```java
leftEdgeX  = camX + rayLength * cos(camHeading + halfFOV)
leftEdgeY  = camY + rayLength * sin(camHeading + halfFOV)
rightEdgeX = camX + rayLength * cos(camHeading - halfFOV)
rightEdgeY = camY + rayLength * sin(camHeading - halfFOV)
```

With a 90-degree FOV, `halfFOV` = 45 degrees. The two edge rays and the camera position
form a V-shape published as a 3-point `Pose2d[]` array.

---

## 5. Interactive Math Visualizations (Desmos)

To help the team understand the math, we provide an interactive Desmos visualization.

### Using the visualization

Open `docs/desmos-tracking-math.html` in any web browser. It contains four interactive
graphs embedded using the Desmos API:

1. **Dot Product Face Visibility** -- Move the robot point around a hub to see which faces
   are visible (green) vs hidden (red). Shows the dot product computation in real time.

2. **atan2 Angle Computation** -- Drag a target point to see how `atan2(dy, dx)` computes
   the field-relative angle. Visualizes the angle arc and the dx/dy components.

3. **Angle Normalization** -- Shows how `atan2(sin(theta), cos(theta))` maps any input
   angle to the [-pi, pi] range. Drag the input angle slider to see the wrapping behavior.

4. **Range Clamping** -- Visualizes the turret's 270-degree range of motion. Drag the
   target angle to see how it clamps to the nearest limit when out of range.

The HTML file is self-contained (uses Desmos's CDN) and works offline once loaded. Share
it with the team or open it during code reviews.

### Desmos LaTeX reference

The key formulas as Desmos-compatible LaTeX (you can paste these directly into
[desmos.com/calculator](https://www.desmos.com/calculator)):

| Formula | Desmos LaTeX | Section |
|---------|-------------|---------|
| Dot product | `d = (f_x - r_x) \cdot n_x + (f_y - r_y) \cdot n_y` | 4.1 |
| Field angle | `\theta_f = \arctan(t_y - r_y, t_x - r_x)` | 4.2 |
| Robot-relative angle | `\theta_r = \theta_f - \theta_h` | 4.3 |
| Normalization | `\theta_n = \arctan(\sin(\theta_r), \cos(\theta_r))` | 4.4 |
| Clamping | `\theta_c = \min(\max(\theta_n, -L), L)` | 4.5 |
| Aim endpoint X | `a_x = r_x + d \cdot \cos(\theta_h + \theta_c)` | 4.6 |
| Aim endpoint Y | `a_y = r_y + d \cdot \sin(\theta_h + \theta_c)` | 4.6 |

---

## 6. WPILib Visualization APIs

### 6.1 Mechanism2d

`Mechanism2d` is a WPILib API for drawing simple 2D mechanisms as a Sendable widget.
It was designed for visualizing robot arms, elevators, and similar mechanisms, but it works
well as an overhead turret dial.

**Class hierarchy**:

```
Mechanism2d                  -- The canvas. A rectangular area with a coordinate system.
  |                             Constructor: Mechanism2d(width, height)
  |
  +-- MechanismRoot2d        -- An anchor point on the canvas.
        |                       Created via: mechanism2d.getRoot("name", x, y)
        |
        +-- MechanismLigament2d  -- A line segment (arm) attached to a root or another
                                    ligament. Has angle, length, line weight, and color.
                                    Created via: root.append(new MechanismLigament2d(...))
```

**Important**: Mechanism2d is a SimGUI/Glass/Shuffleboard widget only. It does NOT
control what AdvantageScope shows on its 2D/3D field views. Those are driven by
`StructPublisher<Pose2d>` and `StructArrayPublisher<Pose2d>` (see section 7).

**Our turret tracker setup**:

```java
Mechanism2d mechanism2d = new Mechanism2d(100, 100);          // 100x100 pixel canvas
MechanismRoot2d root = mechanism2d.getRoot("turret", 50, 50); // root at center

// The turret arm: length 40, starts pointing up (90 deg), thick green line
MechanismLigament2d turretArm = root.append(
    new MechanismLigament2d("arm", 40, 90, 6, new Color8Bit(Color.kGreen)));

// Range limit indicators: shorter, thinner, gray
root.append(new MechanismLigament2d("limitCW", 28, 90-135, 2, new Color8Bit(Color.kGray)));
root.append(new MechanismLigament2d("limitCCW", 28, 90+135, 2, new Color8Bit(Color.kGray)));
```

**Our camera layout setup** (vision system):

```java
Mechanism2d cameraLayoutMech = new Mechanism2d(100, 100);
MechanismRoot2d center = cameraLayoutMech.getRoot("robotCenter", 50, 50);

// Front-right camera: orange line at 60 degrees (90 - 30deg yaw)
center.append(new MechanismLigament2d("frontRightCam", 30, 60, 2, new Color8Bit(Color.kOrange)));
// Front-left camera: yellow line at 120 degrees (90 + 30deg yaw)
center.append(new MechanismLigament2d("frontLeftCam", 30, 120, 2, new Color8Bit(Color.kYellow)));
// Rear camera: cyan line at 270 degrees (90 + 180deg yaw)
center.append(new MechanismLigament2d("rearCam", 30, 270, 2, new Color8Bit(Color.kCyan)));
```

**Coordinate system**: Mechanism2d uses a standard screen coordinate system where
0 degrees = pointing right (east), 90 degrees = pointing up (north). Since we want
"robot forward" to be up on the screen, the turret's 0-degree position maps to
Mechanism2d's 90 degrees. Each frame we set:

```java
turretArm.setAngle(90.0 + turretAngleDegrees);
```

The arm color changes to red when `targetInRange` is false, giving immediate visual
feedback that the target is outside the turret's range.

**How it gets displayed**: We call `Telemetry.putData("TurretTracker/Mechanism", mechanism2d)`
which puts it on NetworkTables as a Sendable. SimGUI, Glass, and Shuffleboard all know
how to render Mechanism2d widgets. In SimGUI, it appears under the "Other Devices" section.

### 6.2 Sendable Protocol

`Mechanism2d` implements `Sendable`, which is WPILib's interface for objects that can
publish themselves to NetworkTables in a standardized way. When you call
`SmartDashboard.putData()` (or our `Telemetry.putData()`), WPILib:

1. Calls `initSendable(SendableBuilder)` on the object
2. The object registers its properties (angles, colors, structure)
3. A background thread keeps the NT values synchronized
4. Dashboard tools (SimGUI, Glass, Shuffleboard) subscribe and render

This is the same mechanism that makes `SubsystemBase`, `PIDController`, `Field2d`, and
other WPILib objects appear as widgets on dashboards.

---

## 7. NetworkTables and AdvantageScope

### 7.1 StructPublisher

WPILib 2024+ introduced **struct-based NetworkTables publishing** for geometry types.
Instead of publishing X, Y, and rotation as separate doubles, you publish an entire
`Pose2d` or `Pose3d` as a single binary-encoded NT entry. AdvantageScope understands
these struct types natively.

```java
NetworkTableInstance nti = NetworkTableInstance.getDefault();

// Single Pose3d
StructPublisher<Pose3d> publisher = nti
    .getStructTopic("TurretTracker/AimPose3d", Pose3d.struct)
    .publish();

// Array of Pose2d
StructArrayPublisher<Pose2d> arrayPublisher = nti
    .getStructArrayTopic("TurretTracker/AimLine", Pose2d.struct)
    .publish();
```

Each frame:
```java
publisher.set(new Pose3d(...));
arrayPublisher.set(new Pose2d[] { startPose, endPose });
```

### 7.2 Turret Tracker NT Keys

| NT Key | Type | Purpose |
|--------|------|---------|
| `TurretTracker/AimPose3d` | `Pose3d` | Robot position + turret aim rotation (for 3D view) |
| `TurretTracker/TargetPose3d` | `Pose3d` | Hub center in 3D space |
| `TurretTracker/AimLine` | `Pose2d[]` | Two-point array: robot position and aim endpoint |
| `TurretTracker/AngleDeg` | `double` | Current turret angle in degrees |
| `TurretTracker/InRange` | `boolean` | Whether target is within range |
| `TurretTracker/DistanceM` | `double` | Distance to hub center |
| `TurretTracker/BlueHubCenter` | `String` | Computed blue hub center "(X, Y)" (LAB level) |
| `TurretTracker/RedHubCenter` | `String` | Computed red hub center "(X, Y)" (LAB level) |
| `TurretTracker/Status` | `String` | "Tracking Hub Center (12.3 deg, 3.4m)" or "Out of Range" |

### 7.3 Vision System NT Keys

| NT Key | Type | Purpose |
|--------|------|---------|
| `Vision/Status` | `String` | Overall status: "Tracking (FR+FL+Rear)", "No Targets", etc. |
| `Vision/TotalTagsDetected` | `int` | Sum of tags seen across all cameras |
| `Vision/FrontRightCamera/Connected` | `boolean` | Camera connection status |
| `Vision/FrontLeftCamera/Connected` | `boolean` | Camera connection status |
| `Vision/RearCamera/Connected` | `boolean` | Camera connection status |
| `Vision/{cam}Camera/TargetCount` | `int` | Number of AprilTags seen by this camera |
| `Vision/{cam}Camera/DetectedTags` | `String` | List of tag IDs: "[25, 26]" |
| `Vision/{cam}Camera/BestTargetID` | `int` | Closest/best tag fiducial ID |
| `Vision/{cam}Camera/EstimateStatus` | `String` | "Accepted", "Rejected", or "No valid estimate" |
| `Vision/FrontRight/FOVCone` | `Pose2d[]` | 3-point V-shape for AdvantageScope field overlay |
| `Vision/FrontLeft/FOVCone` | `Pose2d[]` | 3-point V-shape for AdvantageScope field overlay |
| `Vision/Rear/FOVCone` | `Pose2d[]` | 3-point V-shape for AdvantageScope field overlay |

### 7.4 Telemetry dual-path

Our custom `Telemetry` class has two output paths:

- **`Telemetry.publish(key, value, level)`** writes to NetworkTables (visible live in
  dashboards).
- **`Telemetry.record(key, value, level)`** writes to the DataLog file (`.wpilog`), which
  AdvantageScope can replay offline.
- **`Telemetry.recordPoses(key, poses, level)`** writes a `Pose2d[]` to the DataLog using
  struct encoding.

We use both so the data is available both live (during simulation) and in post-match replay.

---

## 8. Triple-Camera Vision System

### 8.1 Camera Layout

```
                  Robot Front
                    ^
                   / \
     Front-Left   /   \   Front-Right
     yaw=+30deg  /     \  yaw=-30deg
                /       \
               /  60deg  \
              /  overlap  \

     Combined forward coverage: ~150 degrees


                  Robot Rear
                    |
                    |   Rear Camera
                    |   yaw=180deg
                    v

     Rear coverage: ~90 degrees
```

### 8.2 Camera Positions (Transform3d)

Each camera's position is defined as a `Transform3d` from the robot center. The transform
has two parts: a `Translation3d` (where on the robot) and a `Rotation3d` (which way it
points).

| Camera | X (m) | Y (m) | Z (m) | Pitch | Yaw | Color |
|--------|-------|-------|-------|-------|-----|-------|
| Front-Right | +0.30 | -0.25 | +0.25 | -15 deg | -30 deg | Orange |
| Front-Left | +0.30 | +0.25 | +0.25 | -15 deg | +30 deg | Yellow |
| Rear | -0.30 | 0.00 | +0.25 | -15 deg | 180 deg | Cyan |

**Coordinate conventions**:
- **X**: positive = forward, negative = backward
- **Y**: positive = left, negative = right (WPILib convention)
- **Z**: positive = up
- **Pitch**: negative = tilted down (all cameras tilt 15 deg downward)
- **Yaw**: positive = counter-clockwise. 0 = forward, +30 = angled left, -30 = angled right, 180 = facing backward

### 8.3 FOV Visualization

The vision system provides two types of visualization:

**AdvantageScope field overlay** (controlled by `fovVisualizationRayLength` in context):

Each camera publishes a 3-point `Pose2d[]` to NetworkTables. AdvantageScope renders
these as polylines on the 2D field, forming a V-shaped FOV cone for each camera. The
cone rotates with the robot in real time.

To adjust the length of these FOV lines, change `fovVisualizationRayLength` in
`VisionSubsystemContext.java` (default: 2.0 meters).

**Mechanism2d camera layout widget** (visible in Glass/SimGUI only):

A small top-down diagram showing three colored lines radiating from a center point,
representing the direction each camera faces relative to the robot body. This is a
static diagram (doesn't change at runtime) and is purely for reference.

### 8.4 Pose Estimation Pipeline

Each camera independently runs this pipeline every 20ms:

1. **Get latest result** from PhotonCamera
2. **Update PhotonPoseEstimator** with the result
3. **Reject bad estimates**:
   - Single-tag estimates with ambiguity > 0.2 are rejected
   - Estimates where any tag is > 4.0m away are rejected
4. **Calculate dynamic standard deviations**:
   - Multi-tag (2+ tags): base std dev = 0.5 (high trust)
   - Single-tag: base std dev = 4.0 (low trust)
   - Scaled by distance: `stdDev = base * (1 + distance * 0.1)`
   - Heading std dev is always very large (9999999) -- we trust the gyro for heading
5. **Send to drivetrain**: `addVisionMeasurement(pose, timestamp, stdDevs)`

### 8.5 Simulation

In simulation, `VisionSystemSim` creates virtual cameras that "see" AprilTags placed
on the field. Each `PhotonCameraSim` uses the same camera properties (resolution, FOV,
latency) configured in `VisionSubsystemContext`.

**Performance note**: PhotonVision simulation is computationally expensive (~96ms per
camera per loop). With 3 cameras this causes "CommandScheduler loop overrun" warnings.
This is a simulation-only artifact and does not affect real robot performance. To mitigate,
only the front-right camera has optional video streams enabled; front-left and rear have
streams disabled.

---

## 9. File-by-File Implementation Notes

### Turret Tracker Files

#### `TurretTrackerContext.java`

A Lombok `@Data @Builder` configuration class. Follows the same pattern as
`VisionSubsystemContext`, `DrivetrainContext`, and `FuelSubsystemContext`.

- `@Data` generates getters, `equals`, `hashCode`, and `toString`.
- `@Builder` generates a fluent builder: `TurretTrackerContext.builder().turretRangeOfMotionDegrees(180).build()`.
- `@Builder.Default` sets the default value used when the builder doesn't specify a field.

All parameters have sensible defaults. The `defaults()` static method returns a
fully-default instance.

#### `TurretTracker.java`

The core subsystem. Extends `SubsystemBase` which means:
- Its `periodic()` method is called automatically every 20ms by the `CommandScheduler`.
- It participates in command-based requirements (though nothing requires it yet).
- It can be registered as a Sendable for dashboard display.

**Key design decisions**:

- **Hub center is immutable after construction**. The `blueHubCenter` and `redHubCenter`
  are computed once from the field layout and never change. This avoids repeated field
  layout lookups during periodic.

- **Hub-center targeting, not face targeting**. The hub is a top-entry target (basketball
  hoop style). The turret always aims at the geometric center of the hub regardless of
  which side the robot approaches from. This eliminates the need for face selection,
  dot products, and normal vectors.

- **Alliance defaults to blue**. In simulation, `DriverStation.getAlliance()` often returns
  empty until you set it in the DS sim panel. Defaulting to blue means the tracker works
  immediately.

#### `Constants.java` (modified)

Added the `Hub` inner class with tag ID arrays. These are `int[]` and `int[][]` rather
than `List<Integer>` because they're compile-time constants and simpler to declare inline.

The tag IDs were extracted from `2026-rebuilt-welded.json` in the WPILib apriltag jar.
Corner tags (1, 6, 7, 12 for red; 17, 22, 23, 28 for blue) are excluded because they're
at a different height (0.889m vs 1.124m) and aren't useful for turret aiming.

#### `RobotContainer.java` (modified)

Added turret tracker instantiation alongside the other subsystems:

```java
private final TurretTracker turretTracker =
    new TurretTracker(TurretTrackerContext.defaults(), driveTrain);
```

It's created after `driveTrain` (which it depends on) and registered with the telemetry
system via `Telemetry.putData(this.turretTracker)`.

### Vision System Files

#### `VisionSubsystemContext.java`

Lombok `@Data @Builder` configuration for the vision subsystem. Key fields:

- **Camera names**: `frontRightCameraName`, `frontLeftCameraName`, `rearCameraName`
- **Camera transforms**: `frontRightCameraToRobot`, `frontLeftCameraToRobot`, `rearCameraToRobot`
  (each a `Transform3d` with Translation3d + Rotation3d)
- **Sim camera properties**: resolution (960x720), FOV (90 deg), FPS (30), latency (50ms)
- **Quality thresholds**: `poseAmbiguityThreshold` (0.2), `maxPoseEstimationDistance` (4.0m)
- **Std dev factors**: `singleTagStdDevFactor` (4.0), `multiTagStdDevFactor` (0.5)
- **FOV visualization**: `fovVisualizationRayLength` (2.0m), `enableFovVisualization` (true)

#### `VisionSubsystem.java`

The main vision subsystem. Key structure:

- **Three `PhotonCamera` instances** and three `PhotonPoseEstimator` instances
- **`VisionMeasurementConsumer` functional interface**: decouples vision from drivetrain.
  In `RobotContainer`, this is wired to `drivetrain::addVisionMeasurement`.
- **`periodic()`**: Updates camera telemetry and runs pose estimation for all 3 cameras
- **`simulationPeriodic()`**: Updates `VisionSystemSim` with robot pose and refreshes
  FOV cone visualization
- **`processCamera()`**: Extracted helper that handles one camera's pose estimation pipeline
- **`publishCameraFov()`**: Computes field-relative FOV cone edges for one camera
- **`updateSystemStatus()`**: Builds dynamic status string like "Tracking (FR+FL+Rear)"

---

## 10. How to View It In Sim

### SimGUI (Mechanism2d widgets)

1. Run `./gradlew simulateJava`
2. In the SimGUI window, look for **"Other Devices"** or **"NetworkTables"** in the left panel
3. Find `TurretTracker/Mechanism` and drag it into the main area
4. You'll see a turret dial: a green arm that rotates to track the hub, with gray lines
   showing the +/-135 degree limits
5. Find `Vision/CameraLayout` for the top-down camera direction diagram
6. Drive the robot around -- the turret arm follows the hub center
7. When the hub is behind the robot, the arm turns red and pins to the limit

### AdvantageScope (2D field overlay)

1. Open AdvantageScope and connect to `localhost` (or open a `.wpilog` file)
2. Create a new **2D Field** tab
3. Drag `CurrentPoseEstimator` (the robot's pose) onto the field
4. Drag `TurretTracker/AimLine` onto the same field -- it renders as a line
   showing the aim direction
5. Drag `Vision/FrontRight/FOVCone`, `Vision/FrontLeft/FOVCone`, and
   `Vision/Rear/FOVCone` onto the field -- they render as V-shaped FOV cones
   that rotate with the robot
6. For 3D: create a **3D Field** tab, add `TurretTracker/AimPose3d` and
   `TurretTracker/TargetPose3d` as poses

### NetworkTables (raw values)

In Glass, OutlineViewer, or the SimGUI NetworkTables view:

- `TurretTracker/AngleDeg` -- the turret angle (positive = left/CCW)
- `TurretTracker/InRange` -- true when tracking, false when clamped
- `TurretTracker/DistanceM` -- how far the hub center is
- `TurretTracker/Status` -- human-readable string
- `Vision/Status` -- overall vision status (e.g., "Tracking (FR+FL+Rear)")
- `Vision/TotalTagsDetected` -- total AprilTags seen across all cameras
- `Vision/{cam}Camera/DetectedTags` -- per-camera detected tag list

---

## 11. Configuration and Tuning

### Turret Tracker

All parameters are in `TurretTrackerContext` and can be overridden via the builder:

```java
// Example: narrower turret with a longer aim line
TurretTrackerContext.builder()
    .turretRangeOfMotionDegrees(180.0)   // +/- 90 degrees instead of +/- 135
    .aimVectorLengthMeters(5.0)          // longer visualization line
    .build();
```

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `turretRangeOfMotionDegrees` | 270.0 | Total turret sweep. 270 = +/-135 from forward. |
| `turretHeightMeters` | 0.5 | Z-height of the `AimPose3d` in AdvantageScope 3D. |
| `aimVectorLengthMeters` | 3.0 | Length of the drawn aim line (field meters). |
| `mechanism2dSize` | 100.0 | Canvas size for the Mechanism2d widget (pixels). |
| `mechanismArmLength` | 40.0 | Length of the turret arm in the Mechanism2d (pixels). |

### Vision System

All parameters are in `VisionSubsystemContext`:

```java
// Example: tighter rejection thresholds
VisionSubsystemContext.builder()
    .poseAmbiguityThreshold(0.15)
    .maxPoseEstimationDistance(3.0)
    .fovVisualizationRayLength(3.0)   // longer FOV lines on field
    .build();
```

| Parameter | Default | What it controls |
|-----------|---------|-----------------|
| `cameraFovDegrees` | 90.0 | Simulated camera field of view. |
| `cameraFps` | 30 | Simulated camera frame rate. |
| `cameraAvgLatencyMs` | 50.0 | Simulated average processing latency. |
| `poseAmbiguityThreshold` | 0.2 | Max ambiguity for single-tag estimates (lower = stricter). |
| `maxPoseEstimationDistance` | 4.0 | Max tag distance in meters to trust. |
| `singleTagStdDevFactor` | 4.0 | Base std dev for 1-tag estimates (higher = less trust). |
| `multiTagStdDevFactor` | 0.5 | Base std dev for 2+ tag estimates (lower = more trust). |
| `fovVisualizationRayLength` | 2.0 | Length of FOV cone lines on AdvantageScope field (meters). |
| `enableFovVisualization` | true | Toggle FOV cone rendering. |

**Key distinction**: `fovVisualizationRayLength` controls the AdvantageScope field overlay
FOV cones. The `MechanismLigament2d` length in the camera layout widget is a separate
value that only affects the small Glass/SimGUI diagram.

---

## 12. Next Steps

This system provides the **targeting math, vision localization, and visualization**. To
turn it into a functional shoot-while-driving system, the following pieces would be added:

1. **Turret motor subsystem**: A physical motor (likely a TalonFX or SparkMax) that
   receives a target angle from `TurretTracker.getTurretAngleDegrees()` and uses a PID
   controller to move to that angle. It would read a real encoder for feedback.

2. **Shooter command**: A command that checks `turretTracker.isTargetInRange()` and
   `turretTracker.getDistanceToTargetMeters()`, spins up the shooter wheels to the
   correct speed for that distance, and fires when the turret has settled on target.

3. **Distance-to-speed lookup**: A function or interpolation table that maps distance to
   the hub into shooter wheel RPM.

4. **Moving-while-shooting compensation**: Adjusting the aim angle to account for the
   robot's translational velocity (leading the target).

The `TurretTracker` public API is designed for this:

```java
turretTracker.getTurretAngleDegrees()   // feed to turret motor PID setpoint
turretTracker.isTargetInRange()         // gate for "ready to fire"
turretTracker.getDistanceToTargetMeters() // feed to distance-to-RPM lookup
```
