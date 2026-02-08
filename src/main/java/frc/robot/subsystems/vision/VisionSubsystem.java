package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem for AprilTag detection using PhotonVision.
 * Manages three cameras (front-right, front-left, rear), provides pose estimation,
 * supports simulation, and visualizes camera FOV cones.
 *
 * Features:
 * - Triple camera support (front-right, front-left, rear)
 * - PhotonPoseEstimator integration for robot localization
 * - Dynamic standard deviation calculation
 * - Full simulation support with VisionSystemSim
 * - Rejection logic for poor vision estimates
 * - FOV cone visualization for AdvantageScope and Mechanism2d
 */
public class VisionSubsystem extends SubsystemBase {

    /**
     * Functional interface for vision measurement callback.
     * Used to send vision-based pose estimates to the drivetrain.
     */
    @FunctionalInterface
    public interface VisionMeasurementConsumer {
        void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
    }

    private final VisionSubsystemContext context;
    private final Drivetrain drivetrain;
    private final VisionMeasurementConsumer visionMeasurementConsumer;
    private final PhotonCamera frontRightCamera;
    private final PhotonCamera frontLeftCamera;
    private final PhotonCamera rearCamera;

    // Pose estimation
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator frontRightPoseEstimator;
    private final PhotonPoseEstimator frontLeftPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;

    // Simulation (only created in simulation mode)
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontRightCameraSim;
    private PhotonCameraSim frontLeftCameraSim;
    private PhotonCameraSim rearCameraSim;

    // FOV visualization publishers (simulation only)
    private StructArrayPublisher<Pose2d> frontRightFovPublisher;
    private StructArrayPublisher<Pose2d> frontLeftFovPublisher;
    private StructArrayPublisher<Pose2d> rearFovPublisher;
    private Mechanism2d cameraLayoutMech;

    /**
     * Creates a new VisionSubsystem with the provided configuration.
     *
     * @param context Configuration for the vision subsystem
     * @param drivetrain Drivetrain subsystem reference for pose queries
     * @param visionMeasurementConsumer Callback to send vision measurements to pose estimator
     */
    public VisionSubsystem(
            final VisionSubsystemContext context,
            final Drivetrain drivetrain,
            final VisionMeasurementConsumer visionMeasurementConsumer) {
        this.context = Objects.requireNonNull(context, "Context cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "Drivetrain cannot be null");
        this.visionMeasurementConsumer =
                Objects.requireNonNull(visionMeasurementConsumer, "VisionMeasurementConsumer cannot be null");

        // Initialize PhotonVision cameras
        this.frontRightCamera = new PhotonCamera(context.getFrontRightCameraName());
        this.frontLeftCamera = new PhotonCamera(context.getFrontLeftCameraName());
        this.rearCamera = new PhotonCamera(context.getRearCameraName());

        // Load AprilTag field layout from WPILib
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create PhotonPoseEstimators for each camera
        this.frontRightPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getFrontRightCameraToRobot());
        this.frontLeftPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getFrontLeftCameraToRobot());
        this.rearPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRearCameraToRobot());

        // Initialize simulation if enabled
        // NOTE: PhotonVision simulation is expensive (~96ms per loop) and causes "CommandScheduler
        // loop overrun" warnings. This is a sim-only artifact and does not affect real robot performance.
        // To disable, set enableSimulation=false in VisionSubsystemContext.
        if (RobotBase.isSimulation() && context.isEnableSimulation()) {
            initializeSimulation();
        }

        // Initialize FOV visualization if enabled
        if (RobotBase.isSimulation() && context.isEnableFovVisualization()) {
            initializeFovVisualization();
        }

        // Set up initial telemetry values
        Telemetry.publish("Vision/Status", "Initialized", TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontRightCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontLeftCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RearCamera/Connected", false, TelemetryLevel.MATCH);
    }

    /**
     * Initializes simulation components for vision system.
     * Creates VisionSystemSim and PhotonCameraSim instances with realistic properties.
     */
    private void initializeSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        // Configure front-right camera simulation
        SimCameraProperties frontRightProps = createSimCameraProperties();
        frontRightCameraSim = new PhotonCameraSim(frontRightCamera, frontRightProps);
        visionSim.addCamera(frontRightCameraSim, context.getFrontRightCameraToRobot());
        frontRightCameraSim.enableDrawWireframe(true);
        frontRightCameraSim.enableRawStream(context.isEnablePhotonCameraSimStreams());
        frontRightCameraSim.enableProcessedStream(context.isEnablePhotonCameraSimStreams());

        // Configure front-left camera simulation
        SimCameraProperties frontLeftProps = createSimCameraProperties();
        frontLeftCameraSim = new PhotonCameraSim(frontLeftCamera, frontLeftProps);
        visionSim.addCamera(frontLeftCameraSim, context.getFrontLeftCameraToRobot());
        frontLeftCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        frontLeftCameraSim.enableRawStream(false);
        frontLeftCameraSim.enableProcessedStream(false);

        // Configure rear camera simulation
        SimCameraProperties rearProps = createSimCameraProperties();
        rearCameraSim = new PhotonCameraSim(rearCamera, rearProps);
        visionSim.addCamera(rearCameraSim, context.getRearCameraToRobot());
        rearCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        rearCameraSim.enableRawStream(false);
        rearCameraSim.enableProcessedStream(false);

        Telemetry.publish("Vision/Simulation", "Active (3 cameras)", TelemetryLevel.LAB);
    }

    /**
     * Creates SimCameraProperties with settings from context.
     * Shared across all cameras since they use the same hardware.
     */
    private SimCameraProperties createSimCameraProperties() {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(
                context.getCameraResolutionWidth(),
                context.getCameraResolutionHeight(),
                Rotation2d.fromDegrees(context.getCameraFovDegrees()));
        props.setCalibError(context.getCameraCalibError(), context.getCameraCalibErrorStddev());
        props.setFPS(context.getCameraFps());
        props.setAvgLatencyMs(context.getCameraAvgLatencyMs());
        props.setLatencyStdDevMs(context.getCameraLatencyStddevMs());
        return props;
    }

    /**
     * Initializes FOV cone visualization for AdvantageScope and Mechanism2d.
     * Creates NT publishers for Pose2d arrays (rendered as lines on 2D field)
     * and a Mechanism2d showing the top-down camera layout.
     */
    private void initializeFovVisualization() {
        NetworkTableInstance nti = NetworkTableInstance.getDefault();

        frontRightFovPublisher = nti.getStructArrayTopic("Vision/FrontRight/FOVCone", Pose2d.struct)
                .publish();
        frontLeftFovPublisher = nti.getStructArrayTopic("Vision/FrontLeft/FOVCone", Pose2d.struct)
                .publish();
        rearFovPublisher =
                nti.getStructArrayTopic("Vision/Rear/FOVCone", Pose2d.struct).publish();

        // Mechanism2d: top-down camera layout (robot center, 3 directional lines)
        double mechSize = 100.0;
        cameraLayoutMech = new Mechanism2d(mechSize, mechSize);
        MechanismRoot2d center = cameraLayoutMech.getRoot("robotCenter", mechSize / 2.0, mechSize / 2.0);

        // Mechanism2d angles: 0=right, 90=up (forward). Camera yaw is relative to forward.
        // Front-right at yaw=-30deg: mechanism angle = 90 + (-30) = 60
        center.append(new MechanismLigament2d("frontRightCam", 30, 90 - 30, 2, new Color8Bit(Color.kOrange)));
        // Front-left at yaw=+30deg: mechanism angle = 90 + 30 = 120
        center.append(new MechanismLigament2d("frontLeftCam", 30, 90 + 30, 2, new Color8Bit(Color.kYellow)));
        // Rear at yaw=180deg: mechanism angle = 90 + 180 = 270
        center.append(new MechanismLigament2d("rearCam", 30, 270, 2, new Color8Bit(Color.kCyan)));

        Telemetry.putData("Vision/CameraLayout", cameraLayoutMech);
    }

    /**
     * Updates pose estimation from all cameras and sends measurements to drivetrain.
     */
    private void updatePoseEstimation() {
        Pose2d currentPose = drivetrain.getPose2dEstimator();
        frontRightPoseEstimator.setReferencePose(currentPose);
        frontLeftPoseEstimator.setReferencePose(currentPose);
        rearPoseEstimator.setReferencePose(currentPose);

        processCamera(frontRightCamera, frontRightPoseEstimator, "FrontRight");
        processCamera(frontLeftCamera, frontLeftPoseEstimator, "FrontLeft");
        processCamera(rearCamera, rearPoseEstimator, "Rear");
    }

    /**
     * Processes a single camera for pose estimation.
     *
     * @param camera PhotonCamera instance
     * @param poseEstimator PhotonPoseEstimator for this camera
     * @param cameraName Name for telemetry
     */
    private void processCamera(PhotonCamera camera, PhotonPoseEstimator poseEstimator, String cameraName) {
        PhotonPipelineResult result = camera.getLatestResult();
        if (!result.hasTargets()) {
            return;
        }

        Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);

        if (visionEst.isEmpty()) {
            Telemetry.publish(
                    "Vision/" + cameraName + "Camera/EstimateStatus", "No valid estimate", TelemetryLevel.LAB);
            return;
        }

        EstimatedRobotPose estimatedPose = visionEst.get();

        if (shouldRejectEstimate(estimatedPose, result)) {
            Telemetry.publish("Vision/" + cameraName + "Camera/EstimateStatus", "Rejected", TelemetryLevel.LAB);
            return;
        }

        Matrix<N3, N1> stdDevs = calculateVisionStdDevs(estimatedPose, result);

        visionMeasurementConsumer.accept(
                estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, stdDevs);

        Telemetry.publish("Vision/" + cameraName + "Camera/EstimateStatus", "Accepted", TelemetryLevel.MATCH);
        Telemetry.publish(
                "Vision/" + cameraName + "Camera/EstimateX", estimatedPose.estimatedPose.getX(), TelemetryLevel.MATCH);
        Telemetry.publish(
                "Vision/" + cameraName + "Camera/EstimateY", estimatedPose.estimatedPose.getY(), TelemetryLevel.MATCH);
    }

    /**
     * Calculates dynamic standard deviations based on vision conditions.
     * More tags and closer distance = lower std dev (higher trust).
     */
    private Matrix<N3, N1> calculateVisionStdDevs(EstimatedRobotPose estimate, PhotonPipelineResult result) {
        int tagCount = estimate.targetsUsed.size();

        double avgDistance = estimate.targetsUsed.stream()
                .mapToDouble(target ->
                        target.getBestCameraToTarget().getTranslation().getNorm())
                .average()
                .orElse(4.0);

        double baseStdDev;
        if (tagCount >= 2) {
            baseStdDev = context.getMultiTagStdDevFactor();
        } else {
            baseStdDev = context.getSingleTagStdDevFactor();
        }

        double distanceScaling = 1.0 + (avgDistance * context.getDistanceScalingFactor());
        double xyStdDev = baseStdDev * distanceScaling;
        double thetaStdDev = 9999999;

        Telemetry.publish("Vision/StdDev/XY", xyStdDev, TelemetryLevel.LAB);
        Telemetry.publish("Vision/TagCount", tagCount, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/AvgDistance", avgDistance, TelemetryLevel.LAB);

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }

    /**
     * Determines if a vision estimate should be rejected based on quality metrics.
     */
    private boolean shouldRejectEstimate(EstimatedRobotPose estimate, PhotonPipelineResult result) {
        for (var target : estimate.targetsUsed) {
            double distance = target.getBestCameraToTarget().getTranslation().getNorm();
            if (distance > context.getMaxPoseEstimationDistance()) {
                return true;
            }

            if (estimate.targetsUsed.size() == 1) {
                double ambiguity = target.getPoseAmbiguity();
                if (ambiguity > context.getPoseAmbiguityThreshold()) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public void periodic() {
        boolean isSimulation = RobotBase.isSimulation() && visionSim != null;

        boolean frontRightConnected = isSimulation || frontRightCamera.isConnected();
        boolean frontLeftConnected = isSimulation || frontLeftCamera.isConnected();
        boolean rearConnected = isSimulation || rearCamera.isConnected();

        Telemetry.publish("Vision/FrontRightCamera/Connected", frontRightConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontLeftCamera/Connected", frontLeftConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RearCamera/Connected", rearConnected, TelemetryLevel.MATCH);

        PhotonPipelineResult frontRightResult = frontRightCamera.getLatestResult();
        PhotonPipelineResult frontLeftResult = frontLeftCamera.getLatestResult();
        PhotonPipelineResult rearResult = rearCamera.getLatestResult();

        if (frontRightConnected && frontRightResult.hasTargets()) {
            processAndLogTargets("FrontRight", frontRightResult);
        } else {
            Telemetry.publish("Vision/FrontRightCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/FrontRightCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        if (frontLeftConnected && frontLeftResult.hasTargets()) {
            processAndLogTargets("FrontLeft", frontLeftResult);
        } else {
            Telemetry.publish("Vision/FrontLeftCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/FrontLeftCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        if (rearConnected && rearResult.hasTargets()) {
            processAndLogTargets("Rear", rearResult);
        } else {
            Telemetry.publish("Vision/RearCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/RearCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        updateSystemStatus(
                frontRightConnected, frontLeftConnected, rearConnected, frontRightResult, frontLeftResult, rearResult);

        updatePoseEstimation();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) {
            Pose2d robotPose = drivetrain.getPose2dEstimator();
            visionSim.update(robotPose);

            if (context.isEnableFovVisualization()) {
                updateFovVisualization(robotPose);
            }
        }
    }

    /**
     * Computes field-relative FOV cone edges for each camera and publishes
     * as Pose2d arrays for AdvantageScope 2D field overlay.
     * Each FOV cone is a 3-point V shape: [left edge, camera position, right edge].
     */
    private void updateFovVisualization(Pose2d robotPose) {
        double rayLength = context.getFovVisualizationRayLength();
        double halfFovRad = Math.toRadians(context.getCameraFovDegrees() / 2.0);

        publishCameraFov(
                frontRightFovPublisher, robotPose, context.getFrontRightCameraToRobot(), halfFovRad, rayLength);
        publishCameraFov(frontLeftFovPublisher, robotPose, context.getFrontLeftCameraToRobot(), halfFovRad, rayLength);
        publishCameraFov(rearFovPublisher, robotPose, context.getRearCameraToRobot(), halfFovRad, rayLength);
    }

    /**
     * Publishes a single camera's FOV cone as a V-shaped Pose2d array.
     * Projects the camera position and FOV edges onto the field coordinate system.
     */
    private void publishCameraFov(
            StructArrayPublisher<Pose2d> publisher,
            Pose2d robotPose,
            Transform3d cameraToRobot,
            double halfFovRad,
            double rayLength) {
        if (publisher == null) return;

        // Camera position in field coordinates (rotate robot-relative offset by robot heading)
        double robotHeading = robotPose.getRotation().getRadians();
        double cosH = Math.cos(robotHeading);
        double sinH = Math.sin(robotHeading);
        double camX = robotPose.getX() + cameraToRobot.getX() * cosH - cameraToRobot.getY() * sinH;
        double camY = robotPose.getY() + cameraToRobot.getX() * sinH + cameraToRobot.getY() * cosH;

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

        Pose2d leftEdge = new Pose2d(leftX, leftY, new Rotation2d(leftAngle));
        Pose2d camPose = new Pose2d(camX, camY, new Rotation2d(camHeading));
        Pose2d rightEdge = new Pose2d(rightX, rightY, new Rotation2d(rightAngle));

        publisher.set(new Pose2d[] {leftEdge, camPose, rightEdge});
    }

    /**
     * Processes camera results and logs detected AprilTag information.
     */
    private void processAndLogTargets(String cameraName, PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        int targetCount = targets.size();

        Telemetry.publish("Vision/" + cameraName + "Camera/TargetCount", targetCount, TelemetryLevel.MATCH);
        Telemetry.publish(
                "Vision/" + cameraName + "Camera/TimestampSeconds", result.getTimestampSeconds(), TelemetryLevel.LAB);

        List<Integer> detectedTagIds = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0) {
                detectedTagIds.add(fiducialId);

                if (context.isEnableVerboseLogging()) {
                    String prefix = "Vision/" + cameraName + "Camera/Tag" + fiducialId;
                    Telemetry.publish(prefix + "/Yaw", target.getYaw(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Pitch", target.getPitch(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Area", target.getArea(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Skew", target.getSkew(), TelemetryLevel.VERBOSE);
                }
            }
        }

        String tagList = detectedTagIds.isEmpty() ? "None" : detectedTagIds.toString();
        Telemetry.publish("Vision/" + cameraName + "Camera/DetectedTags", tagList, TelemetryLevel.LAB);

        if (!targets.isEmpty()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Telemetry.publish(
                    "Vision/" + cameraName + "Camera/BestTargetID", bestTarget.getFiducialId(), TelemetryLevel.MATCH);
            Telemetry.publish("Vision/" + cameraName + "Camera/BestTargetYaw", bestTarget.getYaw(), TelemetryLevel.LAB);
        }
    }

    /**
     * Updates overall system status telemetry for 3 cameras.
     */
    private void updateSystemStatus(
            boolean frontRightConnected,
            boolean frontLeftConnected,
            boolean rearConnected,
            PhotonPipelineResult frontRightResult,
            PhotonPipelineResult frontLeftResult,
            PhotonPipelineResult rearResult) {

        int connectedCount = 0;
        if (frontRightConnected) connectedCount++;
        if (frontLeftConnected) connectedCount++;
        if (rearConnected) connectedCount++;

        String status;
        if (connectedCount == 0) {
            status = "No Cameras Connected";
        } else if (connectedCount < 3) {
            List<String> offline = new ArrayList<>();
            if (!frontRightConnected) offline.add("FrontRight");
            if (!frontLeftConnected) offline.add("FrontLeft");
            if (!rearConnected) offline.add("Rear");
            status = String.join(", ", offline) + " Offline";
        } else {
            List<String> trackingCams = new ArrayList<>();
            if (frontRightResult.hasTargets()) trackingCams.add("FR");
            if (frontLeftResult.hasTargets()) trackingCams.add("FL");
            if (rearResult.hasTargets()) trackingCams.add("Rear");

            if (trackingCams.isEmpty()) {
                status = "No Targets Detected";
            } else {
                status = "Tracking (" + String.join("+", trackingCams) + ")";
            }
        }

        Telemetry.publish("Vision/Status", status, TelemetryLevel.MATCH);

        int totalTags = 0;
        if (frontRightConnected && frontRightResult.hasTargets()) {
            totalTags += frontRightResult.getTargets().size();
        }
        if (frontLeftConnected && frontLeftResult.hasTargets()) {
            totalTags += frontLeftResult.getTargets().size();
        }
        if (rearConnected && rearResult.hasTargets()) {
            totalTags += rearResult.getTargets().size();
        }
        Telemetry.publish("Vision/TotalTagsDetected", totalTags, TelemetryLevel.MATCH);
    }

    // --- Public API ---

    public PhotonPipelineResult getFrontRightCameraResult() {
        return frontRightCamera.getLatestResult();
    }

    public PhotonPipelineResult getFrontLeftCameraResult() {
        return frontLeftCamera.getLatestResult();
    }

    public PhotonPipelineResult getRearCameraResult() {
        return rearCamera.getLatestResult();
    }

    public PhotonCamera getFrontRightCamera() {
        return frontRightCamera;
    }

    public PhotonCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

    public PhotonCamera getRearCamera() {
        return rearCamera;
    }

    public boolean isFrontRightCameraConnected() {
        return frontRightCamera.isConnected();
    }

    public boolean isFrontLeftCameraConnected() {
        return frontLeftCamera.isConnected();
    }

    public boolean isRearCameraConnected() {
        return rearCamera.isConnected();
    }

    public int getFrontRightTargetCount() {
        PhotonPipelineResult result = frontRightCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    public int getFrontLeftTargetCount() {
        PhotonPipelineResult result = frontLeftCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    public int getRearTargetCount() {
        PhotonPipelineResult result = rearCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }
}
