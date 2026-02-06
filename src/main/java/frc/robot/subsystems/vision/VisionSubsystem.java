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
 * Manages multiple cameras, provides pose estimation, and supports simulation.
 *
 * Features:
 * - Dual camera support (front/rear)
 * - PhotonPoseEstimator integration for robot localization
 * - Dynamic standard deviation calculation
 * - Full simulation support with VisionSystemSim
 * - Rejection logic for poor vision estimates
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
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;

    // Pose estimation
    private final AprilTagFieldLayout fieldLayout;
    private final PhotonPoseEstimator frontPoseEstimator;
    private final PhotonPoseEstimator rearPoseEstimator;

    // Simulation (only created in simulation mode)
    private VisionSystemSim visionSim;
    private PhotonCameraSim frontCameraSim;
    private PhotonCameraSim rearCameraSim;

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
        this.frontCamera = new PhotonCamera(context.getFrontCameraName());
        this.rearCamera = new PhotonCamera(context.getRearCameraName());

        // Load AprilTag field layout from WPILib
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create PhotonPoseEstimators for each camera
        this.frontPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getFrontCameraToRobot());

        this.rearPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRearCameraToRobot());

        // Initialize simulation if enabled
        // NOTE: PhotonVision simulation is expensive (~96ms per loop) and causes "CommandScheduler
        // loop overrun" warnings. This is a sim-only artifact and does not affect real robot performance.
        // To disable, set enableSimulation=false in VisionSubsystemContext.
        if (RobotBase.isSimulation() && context.isEnableSimulation()) {
            initializeSimulation();
        }

        // Set up initial telemetry values
        Telemetry.publish("Vision/Status", "Initialized", TelemetryLevel.MATCH);
        Telemetry.publish("Vision/FrontCamera/Connected", false, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RearCamera/Connected", false, TelemetryLevel.MATCH);
    }

    /**
     * Initializes simulation components for vision system.
     * Creates VisionSystemSim and PhotonCameraSim instances with realistic properties.
     */
    private void initializeSimulation() {
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(fieldLayout);

        // Configure front camera simulation
        SimCameraProperties frontProps = new SimCameraProperties();
        frontProps.setCalibration(
                context.getCameraResolutionWidth(),
                context.getCameraResolutionHeight(),
                Rotation2d.fromDegrees(context.getCameraFovDegrees()));
        frontProps.setCalibError(context.getCameraCalibError(), context.getCameraCalibErrorStddev());
        frontProps.setFPS(context.getCameraFps());
        frontProps.setAvgLatencyMs(context.getCameraAvgLatencyMs());
        frontProps.setLatencyStdDevMs(context.getCameraLatencyStddevMs());

        frontCameraSim = new PhotonCameraSim(frontCamera, frontProps);
        visionSim.addCamera(frontCameraSim, context.getFrontCameraToRobot());
        frontCameraSim.enableDrawWireframe(true);

        frontCameraSim.enableRawStream(context.isEnablePhotonCameraSimStreams());
        frontCameraSim.enableProcessedStream(context.isEnablePhotonCameraSimStreams());

        // Configure rear camera simulation
        SimCameraProperties rearProps = new SimCameraProperties();
        rearProps.setCalibration(
                context.getCameraResolutionWidth(),
                context.getCameraResolutionHeight(),
                Rotation2d.fromDegrees(context.getCameraFovDegrees()));
        rearProps.setCalibError(context.getCameraCalibError(), context.getCameraCalibErrorStddev());
        rearProps.setFPS(context.getCameraFps());
        rearProps.setAvgLatencyMs(context.getCameraAvgLatencyMs());
        rearProps.setLatencyStdDevMs(context.getCameraLatencyStddevMs());

        rearCameraSim = new PhotonCameraSim(rearCamera, rearProps);
        visionSim.addCamera(rearCameraSim, context.getRearCameraToRobot());
        rearCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        rearCameraSim.enableRawStream(false);
        rearCameraSim.enableProcessedStream(false);

        Telemetry.publish("Vision/Simulation", "Active", TelemetryLevel.LAB);
    }

    /**
     * Updates pose estimation from both cameras and sends measurements to drivetrain.
     */
    private void updatePoseEstimation() {
        Pose2d currentPose = drivetrain.getPose2dEstimator();
        frontPoseEstimator.setReferencePose(currentPose);
        rearPoseEstimator.setReferencePose(currentPose);

        processCamera(frontCamera, frontPoseEstimator, "Front");
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
        // Skip isConnected() check - it's expensive and already checked in periodic()
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
     *
     * @param estimate Estimated robot pose
     * @param result Pipeline result
     * @return Standard deviations for x, y, and theta
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
     *
     * @param estimate Estimated robot pose
     * @param result Pipeline result
     * @return true if estimate should be rejected
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
        // In simulation, skip the expensive isConnected() calls - cameras are always available
        boolean isSimulation = RobotBase.isSimulation() && visionSim != null;

        // Update connection status (skip in simulation to avoid performance issues)
        boolean frontConnected = isSimulation || frontCamera.isConnected();
        boolean rearConnected = isSimulation || rearCamera.isConnected();

        Telemetry.publish("Vision/FrontCamera/Connected", frontConnected, TelemetryLevel.MATCH);
        Telemetry.publish("Vision/RearCamera/Connected", rearConnected, TelemetryLevel.MATCH);

        // Get latest results from both cameras
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult rearResult = rearCamera.getLatestResult();

        // Process and log AprilTag detections from front camera
        if (frontConnected && frontResult.hasTargets()) {
            processAndLogTargets("Front", frontResult);
        } else {
            Telemetry.publish("Vision/FrontCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/FrontCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        // Process and log AprilTag detections from rear camera
        if (rearConnected && rearResult.hasTargets()) {
            processAndLogTargets("Rear", rearResult);
        } else {
            Telemetry.publish("Vision/RearCamera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/RearCamera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        // Update overall system status
        updateSystemStatus(frontConnected, rearConnected, frontResult, rearResult);

        // Perform pose estimation and send to drivetrain
        updatePoseEstimation();
    }

    @Override
    public void simulationPeriodic() {
        if (visionSim != null) {
            Pose2d robotPose = drivetrain.getPose2dEstimator();
            visionSim.update(robotPose);
        }
    }

    /**
     * Processes camera results and logs detected AprilTag information.
     *
     * @param cameraName Name of the camera (for logging)
     * @param result Pipeline result from PhotonVision
     */
    private void processAndLogTargets(String cameraName, PhotonPipelineResult result) {
        List<PhotonTrackedTarget> targets = result.getTargets();
        int targetCount = targets.size();

        Telemetry.publish("Vision/" + cameraName + "Camera/TargetCount", targetCount, TelemetryLevel.MATCH);
        Telemetry.publish(
                "Vision/" + cameraName + "Camera/TimestampSeconds", result.getTimestampSeconds(), TelemetryLevel.LAB);

        // Collect AprilTag IDs
        List<Integer> detectedTagIds = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0) { // Valid AprilTag
                detectedTagIds.add(fiducialId);

                // Log detailed target info if verbose logging enabled
                if (context.isEnableVerboseLogging()) {
                    String prefix = "Vision/" + cameraName + "Camera/Tag" + fiducialId;
                    Telemetry.publish(prefix + "/Yaw", target.getYaw(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Pitch", target.getPitch(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Area", target.getArea(), TelemetryLevel.VERBOSE);
                    Telemetry.publish(prefix + "/Skew", target.getSkew(), TelemetryLevel.VERBOSE);
                }
            }
        }

        // Log comma-separated list of detected tag IDs
        String tagList = detectedTagIds.isEmpty() ? "None" : detectedTagIds.toString();
        Telemetry.publish("Vision/" + cameraName + "Camera/DetectedTags", tagList, TelemetryLevel.LAB);

        // Log best target info
        if (!targets.isEmpty()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            Telemetry.publish(
                    "Vision/" + cameraName + "Camera/BestTargetID", bestTarget.getFiducialId(), TelemetryLevel.MATCH);
            Telemetry.publish("Vision/" + cameraName + "Camera/BestTargetYaw", bestTarget.getYaw(), TelemetryLevel.LAB);
        }
    }

    /**
     * Updates overall system status telemetry.
     *
     * @param frontConnected Whether front camera is connected
     * @param rearConnected Whether rear camera is connected
     * @param frontResult Front camera pipeline result
     * @param rearResult Rear camera pipeline result
     */
    private void updateSystemStatus(
            boolean frontConnected,
            boolean rearConnected,
            PhotonPipelineResult frontResult,
            PhotonPipelineResult rearResult) {
        String status;

        if (!frontConnected && !rearConnected) {
            status = "No Cameras Connected";
        } else if (!frontConnected) {
            status = "Front Camera Offline";
        } else if (!rearConnected) {
            status = "Rear Camera Offline";
        } else {
            boolean frontHasTargets = frontResult.hasTargets();
            boolean rearHasTargets = rearResult.hasTargets();

            if (frontHasTargets && rearHasTargets) {
                status = "Tracking (Both Cameras)";
            } else if (frontHasTargets) {
                status = "Tracking (Front Only)";
            } else if (rearHasTargets) {
                status = "Tracking (Rear Only)";
            } else {
                status = "No Targets Detected";
            }
        }

        Telemetry.publish("Vision/Status", status, TelemetryLevel.MATCH);

        // Total tag count across both cameras
        int totalTags = 0;
        if (frontConnected && frontResult.hasTargets()) {
            totalTags += frontResult.getTargets().size();
        }
        if (rearConnected && rearResult.hasTargets()) {
            totalTags += rearResult.getTargets().size();
        }
        Telemetry.publish("Vision/TotalTagsDetected", totalTags, TelemetryLevel.MATCH);
    }

    /**
     * Gets the latest result from the front camera.
     *
     * @return PhotonPipelineResult from front camera
     */
    public PhotonPipelineResult getFrontCameraResult() {
        return frontCamera.getLatestResult();
    }

    /**
     * Gets the latest result from the rear camera.
     *
     * @return PhotonPipelineResult from rear camera
     */
    public PhotonPipelineResult getRearCameraResult() {
        return rearCamera.getLatestResult();
    }

    /**
     * Gets the front PhotonCamera instance.
     * Useful for future pose estimation integration.
     *
     * @return Front PhotonCamera
     */
    public PhotonCamera getFrontCamera() {
        return frontCamera;
    }

    /**
     * Gets the rear PhotonCamera instance.
     * Useful for future pose estimation integration.
     *
     * @return Rear PhotonCamera
     */
    public PhotonCamera getRearCamera() {
        return rearCamera;
    }

    /**
     * Checks if the front camera is connected.
     *
     * @return true if front camera is connected
     */
    public boolean isFrontCameraConnected() {
        return frontCamera.isConnected();
    }

    /**
     * Checks if the rear camera is connected.
     *
     * @return true if rear camera is connected
     */
    public boolean isRearCameraConnected() {
        return rearCamera.isConnected();
    }

    /**
     * Gets the number of AprilTags currently detected by the front camera.
     *
     * @return Number of targets detected by front camera
     */
    public int getFrontTargetCount() {
        PhotonPipelineResult result = frontCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }

    /**
     * Gets the number of AprilTags currently detected by the rear camera.
     *
     * @return Number of targets detected by rear camera
     */
    public int getRearTargetCount() {
        PhotonPipelineResult result = rearCamera.getLatestResult();
        return result.hasTargets() ? result.getTargets().size() : 0;
    }
}
