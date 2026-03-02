package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import lombok.Builder;
import lombok.Data;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/**
 * Configuration context for the Vision subsystem using PhotonVision.
 * Supports four cameras (front-right, front-left, right-side, left-side)
 * for AprilTag detection and localization, with FOV visualization in simulation.
 */
@Data
@Builder
public class VisionSubsystemContext {

    /**
     * Network table name for the right-front camera
     */
    @Builder.Default
    private final String rightFrontCameraName = "pv-right-front";

    /**
     * Network table name for the left-front camera
     */
    @Builder.Default
    private final String leftFrontCameraName = "pv-left-front";

    /**
     * Network table name for the right-side camera
     */
    @Builder.Default
    private final String rightSideCameraName = "pv-right-side";

    /**
     * Network table name for the left-side camera
     */
    @Builder.Default
    private final String leftSideCameraName = "pv-left-side";

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
     * Transform from robot center to right-front camera optical center.
     * Mounted on the front-right corner, co-located with the right-side camera.
     * Position from CAD (2026-03-01): X=13.105in forward, Y=11.615in right, Z=7in up.
     * Position: X=+0.3329m forward, Y=-0.2950m right, Z=+0.1778m up.
     * Rotation: pitch=+15deg (nose up — confirmed from CAD mount design), yaw=-30deg (angled right — confirmed from CAD).
     */
    @Builder.Default
    private final Transform3d robotToRightFrontCamera = new Transform3d(
            new Translation3d(0.3329, -0.2950, 0.1778), new Rotation3d(0, Math.toRadians(15), Math.toRadians(-30)));

    /**
     * Transform from robot center to left-front camera optical center.
     * Mounted on the front-left corner, co-located with the left-side camera.
     * Position from CAD (2026-03-01): X=13.105in forward, Y=11.615in left, Z=7in up.
     * Position: X=+0.3329m forward, Y=+0.2950m left, Z=+0.1778m up.
     * Rotation: pitch=+15deg (nose up — confirmed from CAD mount design), yaw=+30deg (angled left — confirmed from CAD).
     */
    @Builder.Default
    private final Transform3d robotToLeftFrontCamera = new Transform3d(
            new Translation3d(0.3329, 0.2950, 0.1778), new Rotation3d(0, Math.toRadians(15), Math.toRadians(30)));

    /**
     * Transform from robot center to right-side camera optical center.
     * Mounted on the front-right corner, co-located with the front-right camera.
     * Position from CAD (2026-03-01): X=10.618in forward, Y=13.076in right, Z=7in up.
     * Position: X=+0.2697m forward, Y=-0.3321m right, Z=+0.1778m up.
     * Rotation: pitch=+15deg (nose up — confirmed from CAD mount design), yaw=-135deg (angled rear-right — confirmed from CAD).
     */
    @Builder.Default
    private final Transform3d robotToRightSideCamera = new Transform3d(
            new Translation3d(0.2697, -0.3321, 0.1778), new Rotation3d(0, Math.toRadians(15), Math.toRadians(-135)));

    /**
     * Transform from robot center to left-side camera optical center.
     * Mounted on the front-left corner, co-located with the front-left camera.
     * Position from CAD (2026-03-01): X=10.618in forward, Y=13.076in left, Z=7in up.
     * Position: X=+0.2697m forward, Y=+0.3321m left, Z=+0.1778m up.
     * Rotation: pitch=+15deg (nose up — confirmed from CAD mount design), yaw=+135deg (angled rear-left — confirmed from CAD).
     */
    @Builder.Default
    private final Transform3d robotToLeftSideCamera = new Transform3d(
            new Translation3d(0.2697, 0.3321, 0.1778), new Rotation3d(0, Math.toRadians(15), Math.toRadians(135)));

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
    private final int simCameraResolutionWidth = 320;

    /**
     * Camera resolution height in pixels
     */
    @Builder.Default
    private final int simCameraResolutionHeight = 240;

    /**
     * Camera field of view in degrees
     */
    @Builder.Default
    private final double simCameraFovDegrees = 70.0;

    /**
     * Camera calibration error in pixels
     */
    @Builder.Default
    private final double simCameraCalibError = 0.35;

    /**
     * Camera calibration error standard deviation in pixels
     */
    @Builder.Default
    private final double simCameraCalibErrorStddev = 0.10;

    /**
     * Camera frames per second
     */
    @Builder.Default
    private final int simCameraFps = 60;

    /**
     * Average camera latency in milliseconds
     */
    @Builder.Default
    private final double simCameraAvgLatencyMs = 50.0;

    /**
     * Camera latency standard deviation in milliseconds
     */
    @Builder.Default
    private final double simCameraLatencyStddevMs = 15.0;

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

    // FOV visualization

    /**
     * Length of the FOV cone visualization rays in meters
     */
    @Builder.Default
    private final double fovVisualizationRayLength = 2.0;

    /**
     * Whether to enable FOV cone visualization in simulation
     */
    @Builder.Default
    private final boolean enableFovVisualization = true;

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
