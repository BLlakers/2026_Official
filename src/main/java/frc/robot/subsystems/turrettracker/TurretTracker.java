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
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * Simulated turret tracking subsystem that calculates the angle needed to aim
 * at the nearest visible hub face. No physical motor — pure software tracking
 * based on pose estimation.
 *
 * <p>Visualization:
 * <ul>
 *   <li>Mechanism2d: 2D overhead turret dial showing aim angle and range limits</li>
 *   <li>AdvantageScope: Pose2d[] aim line from robot to target face</li>
 *   <li>NetworkTables: Live angle, distance, and status values</li>
 * </ul>
 */
public class TurretTracker extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "TurretTracker";

    /**
     * Represents one face of the hub with its two AprilTag IDs,
     * computed midpoint, and outward-facing normal direction.
     */
    record HubFace(int tag1Id, int tag2Id, Translation2d midpoint, Translation2d normal, String name) {}

    private final TurretTrackerContext context;
    private final Drivetrain drivetrain;
    private final AprilTagFieldLayout fieldLayout;

    // Hub face data (built at construction from field layout)
    private final List<HubFace> blueHubFaces;
    private final List<HubFace> redHubFaces;

    // Computed state (updated each periodic cycle)
    private double turretAngleDegrees = 0.0;
    private double rawAngleDegrees = 0.0;
    private boolean targetInRange = false;
    private double distanceToTargetMeters = 0.0;
    private String activeFaceName = "None";
    private int activeTag1 = -1;
    private int activeTag2 = -1;
    private Translation2d activeTargetPoint = new Translation2d();

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

        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Build hub face data from the field layout
        this.blueHubFaces = buildHubFaces(Constants.Hub.BLUE_FACES, new String[] {"West", "East", "North", "South"});
        this.redHubFaces = buildHubFaces(Constants.Hub.RED_FACES, new String[] {"West", "East", "North", "South"});

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
    }

    /**
     * Builds hub face data from tag ID pairs by looking up positions in the field layout.
     * Computes midpoints and outward-facing normals for each face.
     */
    private List<HubFace> buildHubFaces(int[][] faceTags, String[] faceNames) {
        List<HubFace> faces = new ArrayList<>();

        // First compute the hub center from all face tag midpoints
        double centerX = 0, centerY = 0;
        int count = 0;
        for (int[] tagPair : faceTags) {
            Optional<Pose3d> pose1 = fieldLayout.getTagPose(tagPair[0]);
            Optional<Pose3d> pose2 = fieldLayout.getTagPose(tagPair[1]);
            if (pose1.isPresent() && pose2.isPresent()) {
                centerX += (pose1.get().getX() + pose2.get().getX()) / 2.0;
                centerY += (pose1.get().getY() + pose2.get().getY()) / 2.0;
                count++;
            }
        }
        if (count > 0) {
            centerX /= count;
            centerY /= count;
        }
        Translation2d hubCenter = new Translation2d(centerX, centerY);

        // Build each face
        for (int i = 0; i < faceTags.length; i++) {
            int[] tagPair = faceTags[i];
            Optional<Pose3d> pose1 = fieldLayout.getTagPose(tagPair[0]);
            Optional<Pose3d> pose2 = fieldLayout.getTagPose(tagPair[1]);

            if (pose1.isPresent() && pose2.isPresent()) {
                double midX = (pose1.get().getX() + pose2.get().getX()) / 2.0;
                double midY = (pose1.get().getY() + pose2.get().getY()) / 2.0;
                Translation2d midpoint = new Translation2d(midX, midY);

                // Normal points outward from hub center through the face midpoint
                Translation2d outward = midpoint.minus(hubCenter);
                double norm = outward.getNorm();
                Translation2d normal = (norm > 0.001)
                        ? new Translation2d(outward.getX() / norm, outward.getY() / norm)
                        : new Translation2d(1, 0);

                faces.add(new HubFace(tagPair[0], tagPair[1], midpoint, normal, faceNames[i]));
            }
        }
        return faces;
    }

    @Override
    public void periodic() {
        Pose2d robotPose = drivetrain.getPose2dEstimator();

        // Select hub faces based on alliance
        List<HubFace> hubFaces = resolveHubFaces();

        if (hubFaces.isEmpty()) {
            publishNoTarget();
            return;
        }

        // Find the nearest visible face
        HubFace bestFace = null;
        double bestDistance = Double.MAX_VALUE;

        for (HubFace face : hubFaces) {
            Translation2d robotToFace = face.midpoint().minus(robotPose.getTranslation());
            // Dot product: positive means robot is on the "front" side of this face
            double dot = robotToFace.getX() * face.normal().getX()
                    + robotToFace.getY() * face.normal().getY();

            if (dot > 0) {
                double distance = robotToFace.getNorm();
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestFace = face;
                }
            }
        }

        // Fallback: if no face is visible (shouldn't happen), use nearest face
        if (bestFace == null) {
            for (HubFace face : hubFaces) {
                double distance =
                        face.midpoint().minus(robotPose.getTranslation()).getNorm();
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestFace = face;
                }
            }
        }

        if (bestFace == null) {
            publishNoTarget();
            return;
        }

        // Update active target info
        activeTargetPoint = bestFace.midpoint();
        activeFaceName = bestFace.name();
        activeTag1 = bestFace.tag1Id();
        activeTag2 = bestFace.tag2Id();
        distanceToTargetMeters = bestDistance;

        // Calculate field-relative angle from robot to target face midpoint
        double dx = activeTargetPoint.getX() - robotPose.getX();
        double dy = activeTargetPoint.getY() - robotPose.getY();
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
        updateAdvantageScope(robotPose);
    }

    private List<HubFace> resolveHubFaces() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == Alliance.Blue ? blueHubFaces : redHubFaces;
        }
        // Default to blue if alliance not set (common in sim)
        return blueHubFaces;
    }

    private void updateMechanism2d() {
        // Mechanism2d: 0° = right (east), 90° = up (north/forward)
        // turretAngleDegrees: 0° = robot forward, positive = CCW
        // So mechanism angle = 90 + turretAngleDegrees
        turretArm.setAngle(90.0 + turretAngleDegrees);
        turretArm.setColor(targetInRange ? new Color8Bit(Color.kGreen) : new Color8Bit(Color.kRed));
    }

    private void updateAdvantageScope(Pose2d robotPose) {
        // Field-relative aim direction
        double aimFieldAngleRad = robotPose.getRotation().getRadians() + Units.degreesToRadians(turretAngleDegrees);

        // Aim pose at robot position, pointed toward target
        Pose3d aimPose = new Pose3d(
                robotPose.getX(),
                robotPose.getY(),
                context.getTurretHeightMeters(),
                new Rotation3d(0, 0, aimFieldAngleRad));
        aimPose3dPublisher.set(aimPose);

        // Target face midpoint as a Pose3d
        Pose3d targetPose = new Pose3d(activeTargetPoint.getX(), activeTargetPoint.getY(), 1.124, new Rotation3d());
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

    private void publishNoTarget() {
        targetInRange = false;
        activeFaceName = "None";
        activeTag1 = -1;
        activeTag2 = -1;
        turretAngleDegrees = 0;
        rawAngleDegrees = 0;
        distanceToTargetMeters = 0;
        Telemetry.publish(TELEMETRY_PREFIX + "/Status", "No Target", TelemetryLevel.MATCH);
    }

    private void captureTelemetry(String prefix) {
        // MATCH level - essential tracking data
        Telemetry.record(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/ActiveFace", activeFaceName, TelemetryLevel.MATCH);

        // Publish to NT for live dashboard
        Telemetry.publish(prefix + "/AngleDeg", turretAngleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/InRange", targetInRange, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/DistanceM", distanceToTargetMeters, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/ActiveFace", activeFaceName, TelemetryLevel.MATCH);

        // LAB level - detailed tracking data
        Telemetry.record(prefix + "/RawAngleDeg", rawAngleDegrees, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/ActiveTag1", activeTag1, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/ActiveTag2", activeTag2, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/RangeOfMotionDeg", context.getTurretRangeOfMotionDegrees(), TelemetryLevel.LAB);

        String status = targetInRange
                ? String.format(
                        "Tracking %s (%.1f°, %.1fm)", activeFaceName, turretAngleDegrees, distanceToTargetMeters)
                : String.format("Out of Range - %s (%.1f°)", activeFaceName, rawAngleDegrees);
        Telemetry.publish(prefix + "/Status", status, TelemetryLevel.MATCH);
    }

    // --- PUBLIC API (for future commands, e.g. shoot-while-driving) ---

    /** Current turret angle in degrees (robot-relative, 0 = forward). */
    public double getTurretAngleDegrees() {
        return turretAngleDegrees;
    }

    /** Whether the target is within the turret's range of motion. */
    public boolean isTargetInRange() {
        return targetInRange;
    }

    /** Distance from robot to the tracked hub face midpoint in meters. */
    public double getDistanceToTargetMeters() {
        return distanceToTargetMeters;
    }

    /** Name of the currently tracked hub face (West/East/North/South). */
    public String getActiveFaceName() {
        return activeFaceName;
    }
}
