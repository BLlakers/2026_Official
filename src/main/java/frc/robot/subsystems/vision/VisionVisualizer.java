package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;

/**
 * Publishes camera FOV cone geometry to AdvantageScope via Pose3d struct array publishers.
 *
 * <p>Each camera's FOV is rendered as a V-shaped 3-point array:
 * [left edge, camera position, right edge]. Published to:
 *
 * <ul>
 *   <li>{@code Vision/FrontRight/FOVCone}</li>
 *   <li>{@code Vision/FrontLeft/FOVCone}</li>
 *   <li>{@code Vision/RightSide/FOVCone}</li>
 *   <li>{@code Vision/LeftSide/FOVCone}</li>
 * </ul>
 *
 * <p>{@link #update} is a no-op when {@code enableFovVisualization} is false in the context.
 */
public class VisionVisualizer {

    private final VisionSubsystemContext context;

    private final StructArrayPublisher<Pose3d> frontRightFovPublisher;
    private final StructArrayPublisher<Pose3d> frontLeftFovPublisher;
    private final StructArrayPublisher<Pose3d> rightSideFovPublisher;
    private final StructArrayPublisher<Pose3d> leftSideFovPublisher;

    /**
     * Creates a VisionVisualizer.
     *
     * @param context vision configuration (camera transforms, FOV, visualization settings)
     */
    public VisionVisualizer(VisionSubsystemContext context) {
        this.context = context;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        this.frontRightFovPublisher = nti.getStructArrayTopic("Vision/FrontRight/FOVCone", Pose3d.struct)
                .publish();
        this.frontLeftFovPublisher = nti.getStructArrayTopic("Vision/FrontLeft/FOVCone", Pose3d.struct)
                .publish();
        this.rightSideFovPublisher = nti.getStructArrayTopic("Vision/RightSide/FOVCone", Pose3d.struct)
                .publish();
        this.leftSideFovPublisher = nti.getStructArrayTopic("Vision/LeftSide/FOVCone", Pose3d.struct)
                .publish();
    }

    /**
     * Recomputes and publishes FOV cone arrays for all four cameras based on the current robot pose.
     * No-op when {@code enableFovVisualization} is false in the context.
     *
     * @param robotPose current robot pose on the field
     */
    public void update(Pose2d robotPose) {
        if (!context.isEnableFovVisualization()) {
            return;
        }

        double rayLength = context.getFovVisualizationRayLength();
        double halfFovRad = Math.toRadians(context.getSimCameraFovDegrees() / 2.0);

        publishCameraFov(
                frontRightFovPublisher, robotPose, context.getRobotToRightFrontCamera(), halfFovRad, rayLength);
        publishCameraFov(frontLeftFovPublisher, robotPose, context.getRobotToLeftFrontCamera(), halfFovRad, rayLength);
        publishCameraFov(rightSideFovPublisher, robotPose, context.getRobotToRightSideCamera(), halfFovRad, rayLength);
        publishCameraFov(leftSideFovPublisher, robotPose, context.getRobotToLeftSideCamera(), halfFovRad, rayLength);
    }

    /**
     * Publishes a single camera's FOV cone as a V-shaped Pose3d array.
     * Projects the camera position and FOV edges onto the field coordinate system
     * at the camera's mounted Z height.
     */
    private void publishCameraFov(
            StructArrayPublisher<Pose3d> publisher,
            Pose2d robotPose,
            Transform3d cameraToRobot,
            double halfFovRad,
            double rayLength) {

        // Camera position in field coordinates (rotate robot-relative offset by robot heading)
        double robotHeading = robotPose.getRotation().getRadians();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double camX = robotPose.getX() + cameraToRobot.getX() * cosH - cameraToRobot.getY() * sinH;
        double camY = robotPose.getY() + cameraToRobot.getX() * sinH + cameraToRobot.getY() * cosH;
        double camZ = cameraToRobot.getZ();

        // Camera heading in field coordinates (robot heading + camera yaw)
        double cameraYaw = cameraToRobot.getRotation().getZ();
        double camHeading = robotHeading + cameraYaw;

        // Left and right edges of FOV
        double leftAngle = camHeading + halfFovRad;
        double leftX = camX + rayLength * Math.cos(leftAngle);
        double leftY = camY + rayLength * Math.sin(leftAngle);

        double rightAngle = camHeading - halfFovRad;
        double rightX = camX + rayLength * Math.cos(rightAngle);
        double rightY = camY + rayLength * Math.sin(rightAngle);

        Rotation3d leftRot = new Rotation3d(0, 0, leftAngle);
        Rotation3d camRot = new Rotation3d(0, 0, camHeading);
        Rotation3d rightRot = new Rotation3d(0, 0, rightAngle);

        Pose3d leftEdge = new Pose3d(leftX, leftY, camZ, leftRot);
        Pose3d camPose = new Pose3d(camX, camY, camZ, camRot);
        Pose3d rightEdge = new Pose3d(rightX, rightY, camZ, rightRot);

        publisher.set(new Pose3d[] {leftEdge, camPose, rightEdge});
    }
}
