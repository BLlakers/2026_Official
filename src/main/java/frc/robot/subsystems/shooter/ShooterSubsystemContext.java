package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link ShooterSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → ball fired toward the target</li>
 *   <li>Negative output → reverse to clear jams or eject balls</li>
 * </ul>
 */
@Data
@Builder
public class ShooterSubsystemContext {

    /**
     * Creates a ShooterSubsystemContext with default values from
     * {@link frc.robot.Constants.ShooterConstants}.
     *
     * @return Default configuration context
     */
    public static ShooterSubsystemContext defaults() {
        return ShooterSubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the shooter flywheel motor (NEO on SparkMax). */
    @Builder.Default
    private final int shooterMotorId = SHOOTER_MOTOR_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for the flywheel motor in amps. */
    @Builder.Default
    private final int shooterCurrentLimit = SHOOTER_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratio
    // -------------------------------------------------------------------------

    /** Flywheel gear ratio — direct drive, NEO shaft to flywheel. 1:1. */
    @Builder.Default
    private final double shooterGearRatio = SHOOTER_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Motor inversion
    // -------------------------------------------------------------------------

    /**
     * Whether to invert the flywheel motor.
     * <b>TODO: confirm during first shooter test.</b>
     */
    @Builder.Default
    private final boolean shooterMotorInverted = false;

    // -------------------------------------------------------------------------
    // Open-loop speeds [-1.0, 1.0]
    // Convention: positive = fire ball toward target
    // -------------------------------------------------------------------------

    /**
     * Open-loop speed for the flywheel when shooting. Positive.
     * <b>TODO: replace with physics-solver RPM target after calibration.</b>
     */
    @Builder.Default
    private final double shooterSpeed = SHOOTER_SPEED;

    /**
     * Open-loop speed for the flywheel when reversing. Negative.
     * <b>TODO: tune for effective jam clearing.</b>
     */
    @Builder.Default
    private final double shooterReverseSpeed = SHOOTER_REVERSE_SPEED;
}
