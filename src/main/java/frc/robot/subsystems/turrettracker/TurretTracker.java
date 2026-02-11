package frc.robot.subsystems.turrettracker;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.Optional;
import lombok.Getter;

/**
 * Simulated turret tracking subsystem that calculates the angle needed to aim
 * at a target. No physical motor -- pure software tracking based on pose
 * estimation.
 *
 * <p>Supports two tracking modes:
 * <ul>
 *   <li><b>Shooting</b> (default): aims at the geometric hub center, computed
 *       from AprilTag positions on all four faces. The hub is a top-entry target
 *       (basketball hoop style).</li>
 *   <li><b>Passing</b>: aims at a point between the alliance wall and the hub,
 *       offset north or south depending on the robot's position relative to the
 *       hub Y-center. Used when the robot collects fuel mid-field and passes it
 *       back toward the shooting zone for teammates.</li>
 * </ul>
 *
 * <p>Visualization:
 * <ul>
 *   <li>Mechanism2d: 2D overhead turret dial showing aim angle and range limits</li>
 *   <li>AdvantageScope: Pose2d[] aim line from robot to active target</li>
 *   <li>NetworkTables: Live angle, distance, mode, and status values</li>
 * </ul>
 */
public class TurretTracker extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "TurretTracker";

    private final TurretTrackerContext context;
    private final Drivetrain drivetrain;

    // Hub center positions (computed at construction from field layout)
    private final Translation2d blueHubCenter;
    private final Translation2d redHubCenter;

    // Field dimensions (from AprilTag field layout)
    private final double fieldLengthMeters;
    private final double fieldWidthMeters;

    // Current tracking mode (auto-determined each cycle based on robot position)
    @Getter
    private TrackingMode trackingMode = TrackingMode.SHOOTING;

    private double rawAngleDegrees = 0.0;

    // Current turret angle in degrees (robot-relative, 0 = forward, positive = CCW).
    // Computed state (updated each periodic cycle)
    @Getter
    private double turretAngleDegrees = 0.0;

    // Whether the hub center is within the turret's range of motion.
    @Getter
    private boolean targetInRange = false;

    // Horizontal distance from robot to the active target in meters (2D, X/Y only).
    @Getter
    private double horizontalDistanceMeters = 0.0;

    // 3D distance from turret to the target, accounting for height difference (meters).
    @Getter
    private double distanceToTargetMeters = 0.0;

    // Elevation angle to the target in degrees (positive = upward, 0 = flat).
    // In passing mode this is always 0 (flat lob trajectory).
    @Getter
    private double elevationAngleDegrees = 0.0;

    // The currently resolved target position (hub center or passing target).
    @Getter
    private Translation2d activeTarget = new Translation2d();

    // Visualization: Mechanism2d
    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d turretArm;

    // Visualization: AdvantageScope via StructPublisher
    private final StructPublisher<Pose3d> aimPose3dPublisher;
    private final StructPublisher<Pose3d> targetPose3dPublisher;
    private final StructArrayPublisher<Pose3d> aimLinePublisher;

    public TurretTracker(final TurretTrackerContext context, final Drivetrain drivetrain) {
        this.context = requireNonNull(context, "TurretTrackerContext cannot be null");
        this.drivetrain = requireNonNull(drivetrain, "Drivetrain cannot be null");

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        this.fieldLengthMeters = fieldLayout.getFieldLength();
        this.fieldWidthMeters = fieldLayout.getFieldWidth();

        // Compute hub centers from all face tag positions
        this.blueHubCenter = computeHubCenter(fieldLayout, Constants.Hub.BLUE_FACES);
        this.redHubCenter = computeHubCenter(fieldLayout, Constants.Hub.RED_FACES);

        // Initialize Mechanism2d visualization
        double size = context.getMechanism2dSize();
        this.mechanism2d = new Mechanism2d(size, size);
        MechanismRoot2d root = mechanism2d.getRoot("turret", size / 2.0, size / 2.0);

        // Turret aim arm
        this.turretArm = root.append(
                new MechanismLigament2d("arm", context.getMechanismArmLength(), 90, 6, new Color8Bit(Color.kGreen)));

        // Range limit indicators (thin, faint lines)
        double halfRange = context.getTurretRangeOfMotionDegrees() / 2.0;
        root.append(new MechanismLigament2d(
                "limitCW", context.getMechanismArmLength() * 0.7, 90 - halfRange, 2, new Color8Bit(Color.kGray)));
        root.append(new MechanismLigament2d(
                "limitCCW", context.getMechanismArmLength() * 0.7, 90 + halfRange, 2, new Color8Bit(Color.kGray)));

        // Initialize NT publishers for AdvantageScope
        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        this.aimPose3dPublisher =
                nti.getStructTopic("TurretTracker/AimPose3d", Pose3d.struct).publish();
        this.targetPose3dPublisher =
                nti.getStructTopic("TurretTracker/TargetPose3d", Pose3d.struct).publish();
        this.aimLinePublisher =
                nti.getStructArrayTopic("TurretTracker/AimLine", Pose3d.struct).publish();

        // Register telemetry
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.putData("TurretTracker/Mechanism", mechanism2d);

        Telemetry.publish("TurretTracker/Status", "Initialized", TelemetryLevel.MATCH);
        Telemetry.publish(
                "TurretTracker/BlueHubCenter",
                String.format("(%.3f, %.3f)", blueHubCenter.getX(), blueHubCenter.getY()),
                TelemetryLevel.LAB);
        Telemetry.publish(
                "TurretTracker/RedHubCenter",
                String.format("(%.3f, %.3f)", redHubCenter.getX(), redHubCenter.getY()),
                TelemetryLevel.LAB);
    }

    /**
     * Computes the geometric center of a hub by averaging the midpoints of all
     * face tag pairs. Each face has 2 tags; the hub center is the average of
     * all 4 face midpoints.
     *
     * @param fieldLayout the AprilTag field layout with tag positions
     * @param faceTags array of tag ID pairs, one per face
     * @return the hub center as a Translation2d
     */
    private static Translation2d computeHubCenter(AprilTagFieldLayout fieldLayout, int[][] faceTags) {
        double sumX = 0;
        double sumY = 0;
        int count = 0;

        for (int[] tagPair : faceTags) {
            Optional<Pose3d> pose1 = fieldLayout.getTagPose(tagPair[0]);
            Optional<Pose3d> pose2 = fieldLayout.getTagPose(tagPair[1]);
            if (pose1.isPresent() && pose2.isPresent()) {
                sumX += (pose1.get().getX() + pose2.get().getX()) / 2.0;
                sumY += (pose1.get().getY() + pose2.get().getY()) / 2.0;
                count++;
            }
        }

        if (count == 0) {
            // Should never happen with a valid field layout
            return new Translation2d();
        }
        return new Translation2d(sumX / count, sumY / count);
    }

    @Override
    public void periodic() {
        Pose2d robotPose = drivetrain.getPose2dEstimator();

        // Auto-select tracking mode based on robot position relative to hub
        Translation2d hubCenter = resolveHubCenter();
        trackingMode = isRobotPastHub(robotPose, hubCenter) ? TrackingMode.PASSING : TrackingMode.SHOOTING;

        // Resolve the active target based on tracking mode
        activeTarget = (trackingMode == TrackingMode.PASSING) ? computePassingTarget(robotPose, hubCenter) : hubCenter;

        // Calculate horizontal distance to active target (2D)
        double dx = activeTarget.getX() - robotPose.getX();
        double dy = activeTarget.getY() - robotPose.getY();
        horizontalDistanceMeters = Math.sqrt(dx * dx + dy * dy);

        // Calculate height difference and 3D distance
        double targetZ = (trackingMode == TrackingMode.SHOOTING)
                ? context.getShootingTargetHeightMeters()
                : context.getPassingTargetHeightMeters();
        double dz = targetZ - context.getTurretHeightMeters();
        distanceToTargetMeters = Math.sqrt(dx * dx + dy * dy + dz * dz);

        // Calculate elevation angle (positive = upward, 0 = flat)
        // For passing mode, force flat (0°) since we lob over obstacles
        if (trackingMode == TrackingMode.PASSING) {
            elevationAngleDegrees = 0.0;
        } else {
            elevationAngleDegrees = Units.radiansToDegrees(Math.atan2(dz, horizontalDistanceMeters));
        }

        // Calculate field-relative angle from robot to active target
        double fieldAngleRad = Math.atan2(dy, dx);

        // Convert to robot-relative angle
        double robotHeadingRad = robotPose.getRotation().getRadians();
        double robotRelativeRad = fieldAngleRad - robotHeadingRad;

        // Normalize to [-pi, pi]
        robotRelativeRad = Math.atan2(Math.sin(robotRelativeRad), Math.cos(robotRelativeRad));
        rawAngleDegrees = Units.radiansToDegrees(robotRelativeRad);

        // Clamp to turret range
        double halfRange = context.getTurretRangeOfMotionDegrees() / 2.0;
        targetInRange = Math.abs(rawAngleDegrees) <= halfRange;

        if (targetInRange) {
            turretAngleDegrees = rawAngleDegrees;
        } else {
            turretAngleDegrees = Math.copySign(halfRange, rawAngleDegrees);
        }

        // Update all visualizations
        updateMechanism2d();
        updateAdvantageScope(robotPose, activeTarget);
    }

    private Translation2d resolveHubCenter() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue ? blueHubCenter : redHubCenter;
        }
        // Default to blue if alliance not set (common in sim)
        return blueHubCenter;
    }

    /**
     * Returns true if the robot has crossed past the hub toward the center of the field.
     * "Past" means farther from the alliance wall than the hub is.
     * <ul>
     *   <li>Blue: robot X &gt; hub X (robot is to the right/red-side of hub)</li>
     *   <li>Red: robot X &lt; hub X (robot is to the left/blue-side of hub)</li>
     * </ul>
     */
    private boolean isRobotPastHub(Pose2d robotPose, Translation2d hubCenter) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        if (isRed) {
            return robotPose.getX() < hubCenter.getX();
        } else {
            return robotPose.getX() > hubCenter.getX();
        }
    }

    /**
     * Computes the passing target position based on robot location and alliance.
     *
     * <p>X: midpoint between the alliance wall and the hub center.
     * <ul>
     *   <li>Blue alliance: wall is at X=0, so passX = hubCenter.X / 2</li>
     *   <li>Red alliance: wall is at X=fieldLength, so passX = (fieldLength + hubCenter.X) / 2</li>
     * </ul>
     *
     * <p>Y: the field is divided into two halves at the hub's Y-center.
     * <ul>
     *   <li>Robot north of hub (robotY &gt; hubY): passY = (hubCenter.Y + fieldWidth) / 2</li>
     *   <li>Robot south of hub (robotY &le; hubY): passY = hubCenter.Y / 2</li>
     * </ul>
     */
    private Translation2d computePassingTarget(Pose2d robotPose, Translation2d hubCenter) {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        boolean isRed = alliance.isPresent() && alliance.get() == Alliance.Red;

        // X: midpoint between alliance wall and hub
        double passX;
        if (isRed) {
            passX = (fieldLengthMeters + hubCenter.getX()) / 2.0;
        } else {
            passX = hubCenter.getX() / 2.0;
        }

        // Y: center of the half of the field the robot is on (divided at hub Y)
        double passY;
        if (robotPose.getY() > hubCenter.getY()) {
            // Robot is north of hub — aim at center of northern half
            passY = (hubCenter.getY() + fieldWidthMeters) / 2.0;
        } else {
            // Robot is south of hub — aim at center of southern half
            passY = hubCenter.getY() / 2.0;
        }

        return new Translation2d(passX, passY);
    }

    private void updateMechanism2d() {
        // Mechanism2d: 0 deg = right (east), 90 deg = up (north/forward)
        // turretAngleDegrees: 0 deg = robot forward, positive = CCW
        // So mechanism angle = 90 + turretAngleDegrees
        turretArm.setAngle(90.0 + turretAngleDegrees);

        // Color: green = shooting & in range, yellow = passing & in range, red = out of range
        Color8Bit armColor;
        if (!targetInRange) {
            armColor = new Color8Bit(Color.kRed);
        } else if (trackingMode == TrackingMode.PASSING) {
            armColor = new Color8Bit(Color.kDarkTurquoise);
        } else {
            armColor = new Color8Bit(Color.kGreen);
        }
        turretArm.setColor(armColor);
    }

    private void updateAdvantageScope(Pose2d robotPose, Translation2d hubCenter) {
        // Field-relative aim direction
        double aimFieldAngleRad = robotPose.getRotation().getRadians() + Units.degreesToRadians(turretAngleDegrees);

        // Aim pose at robot position, pointed toward active target with elevation pitch
        double elevPitchRad = Units.degreesToRadians(elevationAngleDegrees);
        Pose3d aimPose = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                context.getTurretHeightMeters(),
                new Rotation3d(0, -elevPitchRad, aimFieldAngleRad));
        aimPose3dPublisher.set(aimPose);

        // Active target as a Pose3d at the actual target height
        double activeTargetZ = (trackingMode == TrackingMode.SHOOTING)
                ? context.getShootingTargetHeightMeters()
                : context.getPassingTargetHeightMeters();
        Pose3d targetPose = new Pose3d(hubCenter.getX(), hubCenter.getY(), activeTargetZ, new Rotation3d());
        targetPose3dPublisher.set(targetPose);

        // Aim line: array of 2 Pose3d from turret to aim vector endpoint.
        // In shooting mode the line pitches upward toward the hub intake height;
        // in passing mode it stays flat (elevation = 0).
        double turretZ = context.getTurretHeightMeters();
        double elevationRad = Units.degreesToRadians(elevationAngleDegrees);
        double aimLength = context.getAimVectorLengthMeters();

        // Horizontal projection of the aim vector (shortened by pitch)
        double horizontalLength = aimLength * Math.cos(elevationRad);
        double endX = robotPose.getX() + horizontalLength * Math.cos(aimFieldAngleRad);
        double endY = robotPose.getY() + horizontalLength * Math.sin(aimFieldAngleRad);
        double endZ = turretZ + aimLength * Math.sin(elevationRad);

        // Rotation3d: roll=0, pitch=-elevation (WPILib pitch is nose-down positive), yaw=aim heading
        Rotation3d aimRot = new Rotation3d(0, -elevationRad, aimFieldAngleRad);
        Pose3d[] aimLine = new Pose3d[] {
            new Pose3d(robotPose.getX(), robotPose.getY(), turretZ, aimRot), new Pose3d(endX, endY, endZ, aimRot),
        };
        aimLinePublisher.set(aimLine);
    }

    private void captureTelemetry(String prefix) {
        // MATCH level - essential tracking data
        Telemetry.record(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/HorizontalDistM", horizontalDistanceMeters, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/ElevationDeg", elevationAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Mode", trackingMode.name(), TelemetryLevel.MATCH);

        // Publish to NT for live dashboard
        Telemetry.publish(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/HorizontalDistM", horizontalDistanceMeters, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/ElevationDeg", elevationAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Mode", trackingMode.name(), TelemetryLevel.MATCH);

        // LAB level - detailed tracking data
        Telemetry.record(prefix + "/RawAngleDeg", rawAngleDegrees, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/RangeOfMotionDeg", context.getTurretRangeOfMotionDegrees(), TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/ActiveTarget",
                String.format("(%.3f, %.3f)", activeTarget.getX(), activeTarget.getY()),
                TelemetryLevel.LAB);

        String modeLabel = trackingMode == TrackingMode.PASSING ? "Passing" : "Hub Center";
        String status = targetInRange
                ? String.format(
                        "Tracking %s (%.1f deg, %.1f elev, %.1fm)",
                        modeLabel, turretAngleDegrees, elevationAngleDegrees, distanceToTargetMeters)
                : String.format("Out of Range (%.1f deg)", rawAngleDegrees);
        Telemetry.publish(prefix + "/Status", status, TelemetryLevel.MATCH);
    }
}
