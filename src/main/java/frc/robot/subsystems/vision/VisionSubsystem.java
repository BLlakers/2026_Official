package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * Vision subsystem for AprilTag detection using PhotonVision.
 * Manages multiple cameras and provides proof-of-life telemetry.
 *
 * This subsystem will be extended for pose estimation and localization
 * in future development iterations.
 */
public class VisionSubsystem extends SubsystemBase {

    private final VisionSubsystemContext context;
    private final Drivetrain drivetrain;
    private final PhotonCamera frontCamera;
    private final PhotonCamera rearCamera;

    /**
     * Creates a new VisionSubsystem with the provided configuration.
     *
     * @param context Configuration for the vision subsystem
     */
    public VisionSubsystem(final VisionSubsystemContext context, final Drivetrain drivetrain) {
        this.context = Objects.requireNonNull(context, "Context cannot be null");
        this.drivetrain = Objects.requireNonNull(drivetrain, "Drivetrain cannot be null");
        // Initialize PhotonVision cameras
        this.frontCamera = new PhotonCamera(context.getFrontCameraName());
        this.rearCamera = new PhotonCamera(context.getRearCameraName());

        // Set up SmartDashboard entries
        SmartDashboard.putString("Vision/Status", "Initialized");
        SmartDashboard.putBoolean("Vision/FrontCamera/Connected", false);
        SmartDashboard.putBoolean("Vision/RearCamera/Connected", false);
    }

    @Override
    public void periodic() {
        // Get latest results from both cameras
        PhotonPipelineResult frontResult = frontCamera.getLatestResult();
        PhotonPipelineResult rearResult = rearCamera.getLatestResult();

        // Update connection status
        boolean frontConnected = frontCamera.isConnected();
        boolean rearConnected = rearCamera.isConnected();

        SmartDashboard.putBoolean("Vision/FrontCamera/Connected", frontConnected);
        SmartDashboard.putBoolean("Vision/RearCamera/Connected", rearConnected);

        // Process and log AprilTag detections from front camera
        if (frontConnected && frontResult.hasTargets()) {
            processAndLogTargets("Front", frontResult);
        } else {
            SmartDashboard.putNumber("Vision/FrontCamera/TargetCount", 0);
            SmartDashboard.putString("Vision/FrontCamera/DetectedTags", "None");
        }

        // Process and log AprilTag detections from rear camera
        if (rearConnected && rearResult.hasTargets()) {
            processAndLogTargets("Rear", rearResult);
        } else {
            SmartDashboard.putNumber("Vision/RearCamera/TargetCount", 0);
            SmartDashboard.putString("Vision/RearCamera/DetectedTags", "None");
        }

        // Update overall system status
        updateSystemStatus(frontConnected, rearConnected, frontResult, rearResult);
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();
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

        SmartDashboard.putNumber("Vision/" + cameraName + "Camera/TargetCount", targetCount);
        SmartDashboard.putNumber("Vision/" + cameraName + "Camera/TimestampSeconds", result.getTimestampSeconds());

        // Collect AprilTag IDs
        List<Integer> detectedTagIds = new ArrayList<>();
        for (PhotonTrackedTarget target : targets) {
            int fiducialId = target.getFiducialId();
            if (fiducialId >= 0) { // Valid AprilTag
                detectedTagIds.add(fiducialId);

                // Log detailed target info if verbose logging enabled
                if (context.isEnableVerboseLogging()) {
                    String prefix = "Vision/" + cameraName + "Camera/Tag" + fiducialId;
                    SmartDashboard.putNumber(prefix + "/Yaw", target.getYaw());
                    SmartDashboard.putNumber(prefix + "/Pitch", target.getPitch());
                    SmartDashboard.putNumber(prefix + "/Area", target.getArea());
                    SmartDashboard.putNumber(prefix + "/Skew", target.getSkew());
                }
            }
        }

        // Log comma-separated list of detected tag IDs
        String tagList = detectedTagIds.isEmpty() ? "None" : detectedTagIds.toString();
        SmartDashboard.putString("Vision/" + cameraName + "Camera/DetectedTags", tagList);

        // Log best target info
        if (!targets.isEmpty()) {
            PhotonTrackedTarget bestTarget = result.getBestTarget();
            SmartDashboard.putNumber("Vision/" + cameraName + "Camera/BestTargetID", bestTarget.getFiducialId());
            SmartDashboard.putNumber("Vision/" + cameraName + "Camera/BestTargetYaw", bestTarget.getYaw());
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

        SmartDashboard.putString("Vision/Status", status);

        // Total tag count across both cameras
        int totalTags = 0;
        if (frontConnected && frontResult.hasTargets()) {
            totalTags += frontResult.getTargets().size();
        }
        if (rearConnected && rearResult.hasTargets()) {
            totalTags += rearResult.getTargets().size();
        }
        SmartDashboard.putNumber("Vision/TotalTagsDetected", totalTags);
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
