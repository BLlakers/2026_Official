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
        this.frontRightCamera = new PhotonCamera(context.getRightFrontCameraName());
        this.frontLeftCamera = new PhotonCamera(context.getLeftFrontCameraName());
        this.rightSideCamera = new PhotonCamera(context.getRightSideCameraName());
        this.leftSideCamera = new PhotonCamera(context.getLeftSideCameraName());

        // Load AprilTag field layout from WPILib
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // Create PhotonPoseEstimators for each camera
        this.frontRightPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRobotToRightFrontCamera());
        this.frontLeftPoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRobotToLeftFrontCamera());
        this.rightSidePoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRobotToRightSideCamera());
        this.leftSidePoseEstimator = new PhotonPoseEstimator(
                fieldLayout, context.getPoseEstimationStrategy(), context.getRobotToLeftSideCamera());

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
        visionSim.addCamera(frontRightCameraSim, context.getRobotToRightFrontCamera());
        frontRightCameraSim.enableDrawWireframe(true);
        frontRightCameraSim.enableRawStream(context.isEnablePhotonCameraSimStreams());
        frontRightCameraSim.enableProcessedStream(context.isEnablePhotonCameraSimStreams());

        // Configure front-left camera simulation
        SimCameraProperties frontLeftProps = createSimCameraProperties();
        frontLeftCameraSim = new PhotonCameraSim(frontLeftCamera, frontLeftProps);
        visionSim.addCamera(frontLeftCameraSim, context.getRobotToLeftFrontCamera());
        frontLeftCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        frontLeftCameraSim.enableRawStream(false);
        frontLeftCameraSim.enableProcessedStream(false);

        // Configure right-side camera simulation
        SimCameraProperties rightSideProps = createSimCameraProperties();
        rightSideCameraSim = new PhotonCameraSim(rightSideCamera, rightSideProps);
        visionSim.addCamera(rightSideCameraSim, context.getRobotToRightSideCamera());
        rightSideCameraSim.enableDrawWireframe(true);
        // Disable video streaming to avoid CameraServer handle issues
        rightSideCameraSim.enableRawStream(false);
        rightSideCameraSim.enableProcessedStream(false);

        // Configure left-side camera simulation
        SimCameraProperties leftSideProps = createSimCameraProperties();
        leftSideCameraSim = new PhotonCameraSim(leftSideCamera, leftSideProps);
        visionSim.addCamera(leftSideCameraSim, context.getRobotToLeftSideCamera());
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
                context.getSimCameraResolutionWidth(),
                context.getSimCameraResolutionHeight(),
                Rotation2d.fromDegrees(context.getSimCameraFovDegrees()));
        props.setCalibError(context.getSimCameraCalibError(), context.getSimCameraCalibErrorStddev());
        props.setFPS(context.getSimCameraFps());
        props.setAvgLatencyMs(context.getSimCameraAvgLatencyMs());
        props.setLatencyStdDevMs(context.getSimCameraLatencyStddevMs());
        return props;
    }

    /**
     * Processes all unread results from a single camera.
     *
     * Telemetry is published from the most recent result in the list so the
     * dashboard always reflects the camera's current state. Pose estimation
     * iterates over every result so no frame is skipped and no stale result
     * is fused twice into the estimator.
     *
     * @param poseEstimator PhotonPoseEstimator for this camera
     * @param cameraName Name used for telemetry keys
     * @param connected Whether this camera is currently connected
     * @param results All unread results from this camera this loop
     */
    private void processCamera(
            PhotonPoseEstimator poseEstimator,
            String cameraName,
            boolean connected,
            List<PhotonPipelineResult> results) {

        if (!connected || results.isEmpty()) {
            Telemetry.publish("Vision/" + cameraName + "Camera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/" + cameraName + "Camera/DetectedTags", "None", TelemetryLevel.LAB);
            return;
        }

        // Telemetry reflects the most recent frame received this loop
        PhotonPipelineResult latestResult = results.get(results.size() - 1);
        if (latestResult.hasTargets()) {
            processAndLogTargets(cameraName, latestResult);
        } else {
            Telemetry.publish("Vision/" + cameraName + "Camera/TargetCount", 0, TelemetryLevel.MATCH);
            Telemetry.publish("Vision/" + cameraName + "Camera/DetectedTags", "None", TelemetryLevel.LAB);
        }

        // Pose estimation processes every frame so none are skipped
        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            Optional<EstimatedRobotPose> visionEst = poseEstimator.update(result);

            if (visionEst.isEmpty()) {
                Telemetry.publish(
                        "Vision/" + cameraName + "Camera/EstimateStatus", "No valid estimate", TelemetryLevel.LAB);
                continue;
            }

            EstimatedRobotPose estimatedPose = visionEst.get();

            if (shouldRejectEstimate(estimatedPose, result)) {
                Telemetry.publish("Vision/" + cameraName + "Camera/EstimateStatus", "Rejected", TelemetryLevel.LAB);
                continue;
            }

            Matrix<N3, N1> stdDevs = calculateVisionStdDevs(estimatedPose, result);

            visionMeasurementConsumer.accept(
                    estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds, stdDevs);

            Telemetry.publish("Vision/" + cameraName + "Camera/EstimateStatus", "Accepted", TelemetryLevel.MATCH);
            Telemetry.publish(
                    "Vision/" + cameraName + "Camera/EstimateX",
                    estimatedPose.estimatedPose.getX(),
                    TelemetryLevel.MATCH);
            Telemetry.publish(
                    "Vision/" + cameraName + "Camera/EstimateY",
                    estimatedPose.estimatedPose.getY(),
                    TelemetryLevel.MATCH);
        }
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

        // Snapshot the current pose estimate once for all estimators.
        // Using a single consistent reference avoids compounding intra-loop updates.
        Pose2d currentPose = drivetrain.getPose2dEstimator();
        frontRightPoseEstimator.setReferencePose(currentPose);
        frontLeftPoseEstimator.setReferencePose(currentPose);
        rightSidePoseEstimator.setReferencePose(currentPose);
        leftSidePoseEstimator.setReferencePose(currentPose);

        // Consume all frames received since the last loop iteration.
        // getAllUnreadResults() ensures each frame is processed exactly once —
        // no frames are skipped and no stale results are fused twice into the estimator.
        List<PhotonPipelineResult> frontRightResults = frontRightCamera.getAllUnreadResults();
        List<PhotonPipelineResult> frontLeftResults = frontLeftCamera.getAllUnreadResults();
        List<PhotonPipelineResult> rightSideResults = rightSideCamera.getAllUnreadResults();
        List<PhotonPipelineResult> leftSideResults = leftSideCamera.getAllUnreadResults();

        processCamera(frontRightPoseEstimator, "FrontRight", frontRightConnected, frontRightResults);
        processCamera(rightSidePoseEstimator, "RightSide", rightSideConnected, rightSideResults);
        //processCamera(leftSidePoseEstimator, "LeftSide", leftSideConnected, leftSideResults);
        //processCamera(frontLeftPoseEstimator, "FrontLeft", frontLeftConnected, frontLeftResults);

        updateSystemStatus(
                frontRightConnected,
                frontLeftConnected,
                rightSideConnected,
                leftSideConnected,
                frontRightResults,
                frontLeftResults,
                rightSideResults,
                leftSideResults);
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
     * Uses the most recent result from each camera's list for status reporting.
     */
    private void updateSystemStatus(
            boolean frontRightConnected,
            boolean frontLeftConnected,
            boolean rightSideConnected,
            boolean leftSideConnected,
            List<PhotonPipelineResult> frontRightResults,
            List<PhotonPipelineResult> frontLeftResults,
            List<PhotonPipelineResult> rightSideResults,
            List<PhotonPipelineResult> leftSideResults) {

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
            if (hasLatestTargets(frontRightResults)) trackingCams.add("FR");
            if (hasLatestTargets(frontLeftResults)) trackingCams.add("FL");
            if (hasLatestTargets(rightSideResults)) trackingCams.add("RS");
            if (hasLatestTargets(leftSideResults)) trackingCams.add("LS");

            if (trackingCams.isEmpty()) {
                status = "No Targets Detected";
            } else {
                status = "Tracking (" + String.join("+", trackingCams) + ")";
            }
        }

        Telemetry.publish("Vision/Status", status, TelemetryLevel.MATCH);

        int totalTags = latestTargetCount(frontRightConnected, frontRightResults)
                + latestTargetCount(frontLeftConnected, frontLeftResults)
                + latestTargetCount(rightSideConnected, rightSideResults)
                + latestTargetCount(leftSideConnected, leftSideResults);
        Telemetry.publish("Vision/TotalTagsDetected", totalTags, TelemetryLevel.MATCH);
    }

    /** Returns true if the most recent result in the list has at least one target. */
    private boolean hasLatestTargets(List<PhotonPipelineResult> results) {
        return !results.isEmpty() && results.get(results.size() - 1).hasTargets();
    }

    /** Returns the target count from the most recent result, or 0 if not connected / no results. */
    private int latestTargetCount(boolean connected, List<PhotonPipelineResult> results) {
        if (!connected || results.isEmpty()) return 0;
        PhotonPipelineResult latest = results.get(results.size() - 1);
        return latest.hasTargets() ? latest.getTargets().size() : 0;
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
