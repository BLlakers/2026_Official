package frc.robot.subsystems.vision;

import lombok.Builder;
import lombok.Data;

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
    private final String frontCameraName;

    /**
     * Network table name for the rear camera
     */
    private final String rearCameraName;

    /**
     * Whether to enable verbose logging to SmartDashboard
     */
    private final boolean enableVerboseLogging;

    /**
     * Whether to enable individual camera telemetry
     */
    private final boolean enableCameraTelemetry;

    /**
     * Creates a default configuration for the Vision subsystem.
     * Uses camera names from Constants and enables verbose logging for development.
     *
     * @return Default VisionSubsystemContext
     */
    public static VisionSubsystemContext defaults() {
        return VisionSubsystemContext.builder()
                .frontCameraName("photonvision-front")
                .rearCameraName("photonvision-rear")
                .enableVerboseLogging(true)
                .enableCameraTelemetry(true)
                .build();
    }
}
