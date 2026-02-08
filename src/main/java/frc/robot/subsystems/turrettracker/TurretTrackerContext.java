package frc.robot.subsystems.turrettracker;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for the TurretTracker subsystem.
 * Controls turret range of motion, visualization parameters, and defaults.
 */
@Data
@Builder
public class TurretTrackerContext {

    /**
     * Total turret range of motion in degrees.
     * 270 means ±135° from robot forward.
     */
    @Builder.Default
    private final double turretRangeOfMotionDegrees = 270.0;

    /**
     * Height of the turret above ground for 3D visualization (meters).
     */
    @Builder.Default
    private final double turretHeightMeters = 0.5;

    /**
     * Length of the aim vector line drawn in visualizations (meters).
     */
    @Builder.Default
    private final double aimVectorLengthMeters = 3.0;

    /**
     * Mechanism2d canvas size in pixels (square).
     */
    @Builder.Default
    private final double mechanism2dSize = 100.0;

    /**
     * Mechanism2d turret arm length in pixels.
     */
    @Builder.Default
    private final double mechanismArmLength = 40.0;

    /**
     * Creates a default configuration.
     */
    public static TurretTrackerContext defaults() {
        return TurretTrackerContext.builder().build();
    }
}
