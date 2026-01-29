package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Builder;
import lombok.Data;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Configuration context for the Vision subsystem using PhotonVision.
 * Supports multiple cameras for AprilTag detection and localization.
 */
@Data
@Builder
public class VisionSubsystemContext {

    /**
     * Network table name for the front camera
     */
    @Builder.Default
    private final String frontCameraName = "photonvision-front";

    /**
     * Network table name for the rear camera
     */
    @Builder.Default
    private final String rearCameraName = "photonvision-rear";

    /**
     * Whether to enable verbose logging to SmartDashboard
     */
    @Builder.Default
    private final boolean enableVerboseLogging = true;

    /**
     * Whether to enable individual camera telemetry
     */
    @Builder.Default
    private final boolean enableCameraTelemetry = true;

    /**
     * Transform from robot center to front camera optical center
     * TODO: Calibrate from CAD
     */
    @Builder.Default
    private final Transform3d frontCameraToRobot =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

    /**
     * Transform from robot center to rear camera optical center
     * TODO: Calibrate from CAD
     */
    @Builder.Default
    private final Transform3d rearCameraToRobot =
            new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, Math.PI));

    /**
     * Whether to enable simulation features (VisionSystemSim)
     */
    @Builder.Default
    private final boolean enableSimulation = true;

    /**
     * Pose estimation strategy for PhotonPoseEstimator
     */
    @Builder.Default
    private final PoseStrategy poseEstimationStrategy = PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    // Simulation camera properties

    /**
     * Camera resolution width in pixels
     */
    @Builder.Default
    private final int cameraResolutionWidth = 960;

    /**
     * Camera resolution height in pixels
     */
    @Builder.Default
    private final int cameraResolutionHeight = 720;

    /**
     * Camera field of view in degrees
     */
    @Builder.Default
    private final double cameraFovDegrees = 90.0;

    /**
     * Camera calibration error in pixels
     */
    @Builder.Default
    private final double cameraCalibError = 0.35;

    /**
     * Camera calibration error standard deviation in pixels
     */
    @Builder.Default
    private final double cameraCalibErrorStddev = 0.10;

    /**
     * Camera frames per second
     */
    @Builder.Default
    private final int cameraFps = 30;

    /**
     * Average camera latency in milliseconds
     */
    @Builder.Default
    private final double cameraAvgLatencyMs = 50.0;

    /**
     * Camera latency standard deviation in milliseconds
     */
    @Builder.Default
    private final double cameraLatencyStddevMs = 15.0;

    // Vision measurement quality parameters

    /**
     * Pose ambiguity threshold - estimates above this are rejected (lower = stricter)
     */
    @Builder.Default
    private final double poseAmbiguityThreshold = 0.2;

    /**
     * Maximum distance in meters to trust vision measurements
     */
    @Builder.Default
    private final double maxPoseEstimationDistance = 4.0;

    /**
     * Standard deviation factor for single-tag estimates (higher = less trust)
     */
    @Builder.Default
    private final double singleTagStdDevFactor = 4.0;

    /**
     * Standard deviation factor for multi-tag estimates (lower = more trust)
     */
    @Builder.Default
    private final double multiTagStdDevFactor = 0.5;

    /**
     * Distance scaling factor for standard deviation calculation
     */
    @Builder.Default
    private final double distanceScalingFactor = 0.1;

    @Builder.Default
    private final boolean enablePhotonCameraSimStreams = false;

    /**
     * Creates a default configuration for the Vision subsystem.
     * All parameters use builder defaults unless overridden.
     *
     * @return Default VisionSubsystemContext with all builder defaults
     */
    public static VisionSubsystemContext defaults() {
        return VisionSubsystemContext.builder().build();
    }
}
