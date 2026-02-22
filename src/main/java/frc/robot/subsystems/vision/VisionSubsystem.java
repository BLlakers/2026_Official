package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotBase;
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
 * Manages four cameras (front-right, front-left, right-side, left-side),
 * provides pose estimation, supports simulation, and visualizes camera FOV cones.
 *
 * Features:
 * - Quad camera support (front-right, front-left, right-side, left-side)
 * - PhotonPoseEstimator integration for robot localization
 * - Dynamic standard deviation calculation
 * - Full simulation support with VisionSystemSim
 * - Rejection logic for poor vision estimates
 * - FOV cone visualization for AdvantageScope via {@link VisionVisualizer}
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
    private final VisionVisualizer visualizer;
    private final PhotonCamera frontRightCamera;
    private final PhotonCamera frontLeftCamera;
    private final PhotonCamera rightSideCamera;
    private final PhotonCamera leftSideCamera;

    // Pose estimation
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator frontRightPoseEstimator;
    private final PhotonPoseEstimator frontLeftPoseEstimator;
    private final PhotonPoseEstimator rightSidePoseEstimator;
    private final PhotonPoseEstimator leftSidePoseEstimator;

    // Simulation (only created in simulation mode)
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontRightCameraSim;
    private PhotonCameraSim frontLeftCameraSim;
    private PhotonCameraSim rightSideCameraSim;
    private PhotonCameraSim leftSideCameraSim;

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
        this.visualizer = new VisionVisualizer(context);

        // Initialize PhotonVision cameras
        this.frontRightCamera = new PhotonCamera(context.getFrontRightCameraName());
        this.frontLeftCamera = new PhotonCamera(context.getFrontLeftCameraName());
        this.rightSideCamera = new PhotonCamera(context.getRightSideCameraName());
        this.leftSideCamera = new PhotonCamera(context.getLeftSideCameraName());

        // Load AprilTag field layout from WPILib
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create PhotonPoseEstimators for each camera
        this.frontRightPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getFrontRightCameraToRobot());
        this.frontLeftPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getFrontLeftCameraToRobot());
        this.rightSidePoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRightSideCameraToRobot());
        this.leftSidePoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getLeftSideCameraToRobot());

        // Initialize simulation if enabled
        // NOTE: PhotonVision simulation is expensive (~96ms per loop) and causes "CommandScheduler
        // loop overrun" warnings. This is a sim-only artifact and does not affect real robot performance.
        // To disable, set enableSimulation=false in VisionSubsystemContext.
        if (RobotBase.isSimulation() && context.isEnableSimulation()) {
            initializeSimulation();
        }

        // Set up initial telemetry values
        Telemetry.publish("Vision/Status", "Initialized", TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontRightCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontLeftCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RightSideCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/LeftSideCamera/Connected", false, TelemetryLevel.MATCH);
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

        // Configure right-side camera simulation
        SimCameraProperties rightSideProps = createSimCameraProperties();
        rightSideCameraSim = new PhotonCameraSim(rightSideCamera, rightSideProps);
        visionSim.addCamera(rightSideCameraSim, context.getRightSideCameraToRobot());
        rightSideCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        rightSideCameraSim.enableRawStream(false);
        rightSideCameraSim.enableProcessedStream(false);

        // Configure left-side camera simulation
        SimCameraProperties leftSideProps = createSimCameraProperties();
        leftSideCameraSim = new PhotonCameraSim(leftSideCamera, leftSideProps);
        visionSim.addCamera(leftSideCameraSim, context.getLeftSideCameraToRobot());
        leftSideCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        leftSideCameraSim.enableRawStream(false);
        leftSideCameraSim.enableProcessedStream(false);

        Telemetry.publish("Vision/Simulation", "Active (4 cameras)", TelemetryLevel.LAB);
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
     * Updates pose estimation from all cameras and sends measurements to drivetrain.
     */
    private void updatePoseEstimation() {
        Pose2d currentPose = drivetrain.getPose2dEstimator();
        frontRightPoseEstimator.setReferencePose(currentPose);
        frontLeftPoseEstimator.setReferencePose(currentPose);
        rightSidePoseEstimator.setReferencePose(currentPose);
        leftSidePoseEstimator.setReferencePose(currentPose);

        processCamera(frontRightCamera, frontRightPoseEstimator, "FrontRight");
        processCamera(frontLeftCamera, frontLeftPoseEstimator, "FrontLeft");
        processCamera(rightSideCamera, rightSidePoseEstimator, "RightSide");
        processCamera(leftSideCamera, leftSidePoseEstimator, "LeftSide");
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
        boolean rightSideConnected = isSimulation || rightSideCamera.isConnected();
        boolean leftSideConnected = isSimulation || leftSideCamera.isConnected();

        Telemetry.publish("Vision/FrontRightCamera/Connected", frontRightConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontLeftCamera/Connected", frontLeftConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RightSideCamera/Connected", rightSideConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/LeftSideCamera/Connected", leftSideConnected, TelemetryLevel.MATCH);

        PhotonPipelineResult frontRightResult = frontRightCamera.getLatestResult();
        PhotonPipelineResult frontLeftResult = frontLeftCamera.getLatestResult();
        PhotonPipelineResult rightSideResult = rightSideCamera.getLatestResult();
        PhotonPipelineResult leftSideResult = leftSideCamera.getLatestResult();

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

        if (rightSideConnected && rightSideResult.hasTargets()) {
            processAndLogTargets("RightSide", rightSideResult);
        } else {
            Telemetry.publish("Vision/RightSideCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/RightSideCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        if (leftSideConnected && leftSideResult.hasTargets()) {
            processAndLogTargets("LeftSide", leftSideResult);
        } else {
            Telemetry.publish("Vision/LeftSideCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/LeftSideCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        updateSystemStatus(
                frontRightConnected,
                frontLeftConnected,
                rightSideConnected,
                leftSideConnected,
                frontRightResult,
                frontLeftResult,
                rightSideResult,
                leftSideResult);

        updatePoseEstimation();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) {
            Pose2d robotPose = drivetrain.getPose2dEstimator();
            visionSim.update(robotPose);
            visualizer.update(robotPose);
        }
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
     * Updates overall system status telemetry for 4 cameras.
     */
    private void updateSystemStatus(
            boolean frontRightConnected,
            boolean frontLeftConnected,
            boolean rightSideConnected,
            boolean leftSideConnected,
            PhotonPipelineResult frontRightResult,
            PhotonPipelineResult frontLeftResult,
            PhotonPipelineResult rightSideResult,
            PhotonPipelineResult leftSideResult) {

        int connectedCount = 0;
        if (frontRightConnected) connectedCount++;
        if (frontLeftConnected) connectedCount++;
        if (rightSideConnected) connectedCount++;
        if (leftSideConnected) connectedCount++;

        String status;
        if (connectedCount == 0) {
            status = "No Cameras Connected";
        } else if (connectedCount < 4) {
            List<String> offline = new ArrayList<>();
            if (!frontRightConnected) offline.add("FrontRight");
            if (!frontLeftConnected) offline.add("FrontLeft");
            if (!rightSideConnected) offline.add("RightSide");
            if (!leftSideConnected) offline.add("LeftSide");
            status = String.join(", ", offline) + " Offline";
        } else {
            List<String> trackingCams = new ArrayList<>();
            if (frontRightResult.hasTargets()) trackingCams.add("FR");
            if (frontLeftResult.hasTargets()) trackingCams.add("FL");
            if (rightSideResult.hasTargets()) trackingCams.add("RS");
            if (leftSideResult.hasTargets()) trackingCams.add("LS");

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
        if (rightSideConnected && rightSideResult.hasTargets()) {
            totalTags += rightSideResult.getTargets().size();
        }
        if (leftSideConnected && leftSideResult.hasTargets()) {
            totalTags += leftSideResult.getTargets().size();
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

    public PhotonPipelineResult getRightSideCameraResult() {
        return rightSideCamera.getLatestResult();
    }

    public PhotonPipelineResult getLeftSideCameraResult() {
        return leftSideCamera.getLatestResult();
    }

    public PhotonCamera getFrontRightCamera() {
        return frontRightCamera;
    }

    public PhotonCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

    public PhotonCamera getRightSideCamera() {
        return rightSideCamera;
    }

    public PhotonCamera getLeftSideCamera() {
        return leftSideCamera;
    }

    public boolean isFrontRightCameraConnected() {
        return frontRightCamera.isConnected();
    }

    public boolean isFrontLeftCameraConnected() {
        return frontLeftCamera.isConnected();
    }

    public boolean isRightSideCameraConnected() {
        return rightSideCamera.isConnected();
    }

    public boolean isLeftSideCameraConnected() {
        return leftSideCamera.isConnected();
    }

    public int getFrontRightTargetCount() {
        PhotonPipelineResult result = frontRightCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    public int getFrontLeftTargetCount() {
        PhotonPipelineResult result = frontLeftCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    public int getRightSideTargetCount() {
        PhotonPipelineResult result = rightSideCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    public int getLeftSideTargetCount() {
        PhotonPipelineResult result = leftSideCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }
}
