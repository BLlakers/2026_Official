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

/**
 * Simulated turret tracking subsystem that calculates the angle needed to aim
 * at the hub center. No physical motor -- pure software tracking based on pose
 * estimation.
 *
 * <p>The hub is a top-entry target (like a basketball hoop). The turret always
 * aims at the geometric center of the hub, computed from AprilTag positions on
 * all four faces. Face selection is not needed because the ball enters from the
 * top, not through a side opening.
 *
 * <p>Visualization:
 * <ul>
 *   <li>Mechanism2d: 2D overhead turret dial showing aim angle and range limits</li>
 *   <li>AdvantageScope: Pose2d[] aim line from robot to hub center</li>
 *   <li>NetworkTables: Live angle, distance, and status values</li>
 * </ul>
 */
public class TurretTracker extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "TurretTracker";

    private final TurretTrackerContext context;
    private final Drivetrain drivetrain;

    // Hub center positions (computed at construction from field layout)
    private final Translation2d blueHubCenter;
    private final Translation2d redHubCenter;

    // Computed state (updated each periodic cycle)
    private double turretAngleDegrees = 0.0;
    private double rawAngleDegrees = 0.0;
    private boolean targetInRange = false;
    private double distanceToTargetMeters = 0.0;

    // Visualization: Mechanism2d
    private final Mechanism2d mechanism2d;
    private final MechanismLigament2d turretArm;

    // Visualization: AdvantageScope via StructPublisher
    private final StructPublisher<Pose3d> aimPose3dPublisher;
    private final StructPublisher<Pose3d> targetPose3dPublisher;
    private final StructArrayPublisher<Pose2d> aimLinePublisher;

    public TurretTracker(final TurretTrackerContext context, final Drivetrain drivetrain) {
        this.context = requireNonNull(context, "TurretTrackerContext cannot be null");
        this.drivetrain = requireNonNull(drivetrain, "Drivetrain cannot be null");

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

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
                nti.getStructArrayTopic("TurretTracker/AimLine", Pose2d.struct).publish();

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

        // Select hub center based on alliance
        Translation2d hubCenter = resolveHubCenter();

        // Calculate distance to hub center
        double dx = hubCenter.getX() - robotPose.getX();
        double dy = hubCenter.getY() - robotPose.getY();
        distanceToTargetMeters = Math.sqrt(dx * dx + dy * dy);

        // Calculate field-relative angle from robot to hub center
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
        updateAdvantageScope(robotPose, hubCenter);
    }

    private Translation2d resolveHubCenter() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue ? blueHubCenter : redHubCenter;
        }
        // Default to blue if alliance not set (common in sim)
        return blueHubCenter;
    }

    private void updateMechanism2d() {
        // Mechanism2d: 0 deg = right (east), 90 deg = up (north/forward)
        // turretAngleDegrees: 0 deg = robot forward, positive = CCW
        // So mechanism angle = 90 + turretAngleDegrees
        turretArm.setAngle(90.0 + turretAngleDegrees);
        turretArm.setColor(targetInRange ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
    }

    private void updateAdvantageScope(Pose2d robotPose, Translation2d hubCenter) {
        // Field-relative aim direction
        double aimFieldAngleRad = robotPose.getRotation().getRadians() + Units.degreesToRadians(turretAngleDegrees);

        // Aim pose at robot position, pointed toward hub center
        Pose3d aimPose = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                context.getTurretHeightMeters(),
                new Rotation3d(0, 0, aimFieldAngleRad));
        aimPose3dPublisher.set(aimPose);

        // Hub center as a Pose3d (Z = turret height for visual alignment)
        Pose3d targetPose =
                new Pose3d(hubCenter.getX(), hubCenter.getY(), context.getTurretHeightMeters(), new Rotation3d());
        targetPose3dPublisher.set(targetPose);

        // Aim line: array of 2 Pose2d (start at robot, end at aim vector endpoint)
        double endX = robotPose.getX() + context.getAimVectorLengthMeters() * Math.cos(aimFieldAngleRad);
        double endY = robotPose.getY() + context.getAimVectorLengthMeters() * Math.sin(aimFieldAngleRad);

        Pose2d[] aimLine = new Pose2d[] {
            robotPose, new Pose2d(endX, endY, new Rotation2d(aimFieldAngleRad)),
        };
        aimLinePublisher.set(aimLine);

        // Also record for DataLog (AdvantageScope replay)
        Telemetry.recordPoses(TELEMETRY_PREFIX + "/AimLine", aimLine, TelemetryLevel.MATCH);
    }

    private void captureTelemetry(String prefix) {
        // MATCH level - essential tracking data
        Telemetry.record(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);

        // Publish to NT for live dashboard
        Telemetry.publish(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);

        // LAB level - detailed tracking data
        Telemetry.record(prefix + "/RawAngleDeg", rawAngleDegrees, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/RangeOfMotionDeg", context.getTurretRangeOfMotionDegrees(), TelemetryLevel.LAB);

        String status = targetInRange
                ? String.format("Tracking Hub Center (%.1f deg, %.1fm)", turretAngleDegrees, distanceToTargetMeters)
                : String.format("Out of Range (%.1f deg)", rawAngleDegrees);
        Telemetry.publish(prefix + "/Status", status, TelemetryLevel.MATCH);
    }

    // --- PUBLIC API (for future commands, e.g. shoot-while-driving) ---

    /** Current turret angle in degrees (robot-relative, 0 = forward, positive = CCW). */
    public double getTurretAngleDegrees() {
        return turretAngleDegrees;
    }

    /** Whether the hub center is within the turret's range of motion. */
    public boolean isTargetInRange() {
        return targetInRange;
    }

    /** Distance from robot to the hub center in meters. */
    public double getDistanceToTargetMeters() {
        return distanceToTargetMeters;
    }
}
