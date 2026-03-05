package frc.robot.subsystems.turret;

import static frc.robot.Constants.TurretConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link TurretSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The turret rotates the shooter assembly horizontally to aim at the hub. A single NEO
 * drives a 20:1 gearbox, whose output shaft drives an external ring gear / pinion stage
 * (ratio TBD from CAD). The SparkMax built-in encoder tracks motor rotations; dividing by
 * the total gear ratio converts to turret degrees.
 *
 * <h2>Gear Ratio</h2>
 * <p>{@code turretGearRatio} = 9 × (74/44) × (120/30) = 666/11 ≈ 60.55:1 (confirmed from CAD).
 * Stage-by-stage breakdown is documented in {@link frc.robot.Constants.TurretConstants#TURRET_GEAR_RATIO}.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>0 = home (turret aimed straight forward)</li>
 *   <li>Positive = counterclockwise (left)</li>
 *   <li>Negative = clockwise (right)</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code turretMotorInverted} — confirm positive output rotates the turret
 *       counterclockwise (left) during first motor test</li>
 *   <li>{@code turretJogSpeed} — tune for safe bring-up speed</li>
 *   <li>{@code turretPositionToleranceDegrees} — tighten after PID tuning</li>
 * </ul>
 */
@Data
@Builder
public class TurretSubsystemContext {

    /**
     * Creates a TurretSubsystemContext with default values from
     * {@link frc.robot.Constants.TurretConstants}.
     *
     * @return Default configuration context
     */
    public static TurretSubsystemContext defaults() {
        return TurretSubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the turret rotation motor (NEO on SparkMax). */
    @Builder.Default
    private final int turretMotorId = TURRET_MOTOR_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for the turret motor in amps. */
    @Builder.Default
    private final int turretCurrentLimit = TURRET_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratio
    // -------------------------------------------------------------------------

    /**
     * Total effective gear ratio between the NEO shaft and the turret output.
     * Currently set to the motor-side gearbox only (20:1).
     * <b>TODO: update to gearbox × ring gear ratio once ring gear geometry is confirmed from CAD.</b>
     */
    @Builder.Default
    private final double turretGearRatio = TURRET_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Range of motion — asymmetric (turret home is not centered in its arc)
    // -------------------------------------------------------------------------

    /**
     * Maximum left (CCW / positive) travel from the home position in degrees.
     * Used for soft limit enforcement in jog and track commands.
     * Must match {@link frc.robot.subsystems.turrettracker.TurretTrackerContext#maxLeftDegrees}.
     */
    @Builder.Default
    private final double maxLeftDegrees = TURRET_MAX_LEFT_DEGREES;

    /**
     * Maximum right (CW) travel from the home position in degrees (positive magnitude).
     * The minimum turret angle is {@code -maxRightDegrees}.
     * Must match {@link frc.robot.subsystems.turrettracker.TurretTrackerContext#maxRightDegrees}.
     */
    @Builder.Default
    private final double maxRightDegrees = TURRET_MAX_RIGHT_DEGREES;

    // -------------------------------------------------------------------------
    // Motor inversion — confirm with build team
    // -------------------------------------------------------------------------

    /**
     * Whether to invert the turret motor.
     * Positive output must rotate the turret counterclockwise (left) — matching the
     * WPILib angle convention used by TurretTracker.
     * <b>TODO: confirm during first motor test.</b>
     */
    @Builder.Default
    private final boolean turretMotorInverted = false;

    // -------------------------------------------------------------------------
    // Manual jog speed — for initial bring-up testing only
    // -------------------------------------------------------------------------

    /**
     * Open-loop speed used by manual jog commands during bring-up.
     * Kept slow to avoid hitting mechanical stops at speed.
     * <b>TODO: remove or gate behind a test mode once closed-loop tracking is working.</b>
     */
    @Builder.Default
    private final double turretJogSpeed = TURRET_JOG_SPEED;

    // -------------------------------------------------------------------------
    // Position tolerance
    // -------------------------------------------------------------------------

    /**
     * Acceptable position error (degrees) for on-target checks.
     * <b>TODO: tighten after PID tuning with the physical robot.</b>
     */
    @Builder.Default
    private final double turretPositionToleranceDegrees = TURRET_POSITION_TOLERANCE_DEGREES;
}
