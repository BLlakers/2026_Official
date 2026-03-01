package frc.robot.subsystems.turrettracker;

import static frc.robot.Constants.TurretConstants.*;

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
     * Maximum left (CCW / positive) turret travel in degrees from home.
     * The turret range is asymmetric — left and right limits differ.
     * Must match {@link frc.robot.subsystems.turret.TurretSubsystemContext#maxLeftDegrees}.
     */
    @Builder.Default
    private final double maxLeftDegrees = TURRET_MAX_LEFT_DEGREES;

    /**
     * Maximum right (CW) turret travel in degrees from home (positive magnitude).
     * The minimum turret angle is {@code -maxRightDegrees}.
     * Must match {@link frc.robot.subsystems.turret.TurretSubsystemContext#maxRightDegrees}.
     */
    @Builder.Default
    private final double maxRightDegrees = TURRET_MAX_RIGHT_DEGREES;

    /**
     * Height of the turret above ground for 3D visualization (meters).
     * 19 inches = 0.4826m.
     */
    @Builder.Default
    private final double turretHeightMeters = 0.4826;

    /**
     * Height of the hub intake opening above ground (meters).
     * 72 inches = 1.8288m. Used in shooting mode to compute elevation angle
     * and 3D distance for motor speed derivation.
     */
    @Builder.Default
    private final double shootingTargetHeightMeters = 1.8288;

    /**
     * Height of the passing target above ground (meters).
     * Passing uses a lob trajectory, so elevation is computed as 0 (flat)
     * rather than aiming down at the ground.
     */
    @Builder.Default
    private final double passingTargetHeightMeters = 0.0;

    /**
     * Length of the aim vector line drawn in visualizations (meters).
     */
    @Builder.Default
    private final double aimVectorLengthMeters = 3.0;

    /**
     * Creates a default configuration.
     */
    public static TurretTrackerContext defaults() {
        return TurretTrackerContext.builder().build();
    }
}
