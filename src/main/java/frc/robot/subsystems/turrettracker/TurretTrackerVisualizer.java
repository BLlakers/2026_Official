package frc.robot.subsystems.turrettracker;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Publishes TurretTracker 3D geometry to AdvantageScope via Pose3d struct publishers.
 *
 * <ul>
 *   <li>{@code TurretTracker/AimPose3d} — turret pose at robot position, pitched toward target</li>
 *   <li>{@code TurretTracker/TargetPose3d} — active target position at target height</li>
 *   <li>{@code TurretTracker/AimLine} — 2-point line from turret to aim vector endpoint</li>
 * </ul>
 */
public class TurretTrackerVisualizer {

    private final TurretTrackerContext context;

    private final StructPublisher<Pose3d> aimPose3dPublisher;
    private final StructPublisher<Pose3d> targetPose3dPublisher;
    private final StructArrayPublisher<Pose3d> aimLinePublisher;

    /**
     * Creates a TurretTrackerVisualizer.
     *
     * @param context turret configuration (geometry, heights)
     */
    public TurretTrackerVisualizer(TurretTrackerContext context) {
        this.context = context;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        this.aimPose3dPublisher =
                nti.getStructTopic("TurretTracker/AimPose3d", Pose3d.struct).publish();
        this.targetPose3dPublisher =
                nti.getStructTopic("TurretTracker/TargetPose3d", Pose3d.struct).publish();
        this.aimLinePublisher =
                nti.getStructArrayTopic("TurretTracker/AimLine", Pose3d.struct).publish();
    }

    /**
     * Recomputes and publishes the aim pose, target pose, and aim line.
     *
     * @param robotPose current robot pose on the field
     * @param activeTarget field position of the active target (hub center or passing target)
     * @param turretAngleDegrees current turret angle in degrees (robot-relative, 0=forward,
     *     positive=CCW)
     * @param elevationAngleDegrees elevation angle in degrees (positive=upward, 0=flat)
     * @param trackingMode current tracking mode (SHOOTING or PASSING)
     */
    public void update(
            Pose2d robotPose,
            Translation2d activeTarget,
            double turretAngleDegrees,
            double elevationAngleDegrees,
            TrackingMode trackingMode) {

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
        Pose3d targetPose = new Pose3d(activeTarget.getX(), activeTarget.getY(), activeTargetZ, new Rotation3d());
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
}
