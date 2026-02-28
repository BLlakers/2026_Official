package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link ShooterSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The shooter is a differential-velocity dual-roller launcher. Two flywheels of different
 * diameters grip opposite sides of the 5.91" fuel ball as it passes through a fixed-angle
 * channel (~23°). By independently controlling the speed of each flywheel, the mechanism
 * controls both exit velocity and backspin rate (Magnus lift effect), which together with
 * the fixed launch angle fully determine the ball's trajectory.
 *
 * <h2>Flywheel Geometry</h2>
 * <ul>
 *   <li>Front flywheel (A): 3.0" diameter (0.0762 m)</li>
 *   <li>Rear flywheel (B): 4.0" diameter (0.1016 m)</li>
 * </ul>
 *
 * <p>At equal RPM, the rear wheel has 1.33× the surface speed of the front wheel due to the
 * diameter difference. Independent speed control exploits this asymmetry to tune backspin.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → ball fired toward the target</li>
 *   <li>Negative output → reverse to clear jams or eject balls</li>
 * </ul>
 *
 * <p>One flywheel motor must be inverted because the wheels grip opposite sides of the ball.
 * The correct inversion must be confirmed during first-run testing.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code shooterFrontMotorInverted} / {@code shooterRearMotorInverted} — confirm
 *       which motor(s) must be inverted so that positive output fires the ball forward</li>
 *   <li>{@code shooterFrontSpeed} / {@code shooterRearSpeed} — current values are open-loop
 *       stubs; replace with physics-solver RPM targets after SHOOTER.md calibration sessions</li>
 *   <li>{@code shooterFrontReverseSpeed} / {@code shooterRearReverseSpeed} — tune for
 *       effective jam clearing</li>
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

    /** CAN ID of the front flywheel motor — flywheel A, 3" diameter (NEO on SparkMax). */
    @Builder.Default
    private final int shooterFrontMotorId = SHOOTER_FRONT_MOTOR_ID;

    /** CAN ID of the rear flywheel motor — flywheel B, 4" diameter (NEO on SparkMax). */
    @Builder.Default
    private final int shooterRearMotorId = SHOOTER_REAR_MOTOR_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for both flywheel motors in amps. */
    @Builder.Default
    private final int shooterCurrentLimit = SHOOTER_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratios — both flywheels are direct-drive
    // -------------------------------------------------------------------------

    /** Front flywheel gear ratio — direct drive, NEO shaft to 3" flywheel. 1:1. */
    @Builder.Default
    private final double shooterFrontGearRatio = SHOOTER_FRONT_GEAR_RATIO;

    /** Rear flywheel gear ratio — direct drive, NEO shaft to 4" flywheel. 1:1. */
    @Builder.Default
    private final double shooterRearGearRatio = SHOOTER_REAR_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Motor inversion — confirm with build team
    // -------------------------------------------------------------------------

    /**
     * Whether to invert the front flywheel motor (flywheel A, 3").
     * The two wheels grip opposite sides of the ball, so exactly one motor
     * must be inverted for both to push the ball in the same direction.
     * <b>TODO: confirm during first shooter test.</b>
     */
    @Builder.Default
    private final boolean shooterFrontMotorInverted = false;

    /**
     * Whether to invert the rear flywheel motor (flywheel B, 4").
     * The two wheels grip opposite sides of the ball, so exactly one motor
     * must be inverted for both to push the ball in the same direction.
     * <b>TODO: confirm during first shooter test.</b>
     */
    @Builder.Default
    private final boolean shooterRearMotorInverted = false;

    // -------------------------------------------------------------------------
    // Open-loop speeds [-1.0, 1.0]
    // Convention: positive = fire ball toward target
    // These are stubs — replace with physics-solver RPM targets after calibration.
    // -------------------------------------------------------------------------

    /**
     * Open-loop speed for the front flywheel (A, 3") when shooting. Positive.
     * <b>TODO: replace with physics-solver RPM target after SHOOTER.md calibration.</b>
     */
    @Builder.Default
    private final double shooterFrontSpeed = SHOOTER_FRONT_SPEED;

    /**
     * Open-loop speed for the rear flywheel (B, 4") when shooting. Positive.
     * <b>TODO: replace with physics-solver RPM target after SHOOTER.md calibration.</b>
     */
    @Builder.Default
    private final double shooterRearSpeed = SHOOTER_REAR_SPEED;

    /**
     * Open-loop speed for the front flywheel when reversing. Negative.
     * <b>TODO: tune for effective jam clearing.</b>
     */
    @Builder.Default
    private final double shooterFrontReverseSpeed = SHOOTER_FRONT_REVERSE_SPEED;

    /**
     * Open-loop speed for the rear flywheel when reversing. Negative.
     * <b>TODO: tune for effective jam clearing.</b>
     */
    @Builder.Default
    private final double shooterRearReverseSpeed = SHOOTER_REAR_REVERSE_SPEED;
}
