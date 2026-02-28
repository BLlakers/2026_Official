package frc.robot.subsystems.intake;

import static frc.robot.Constants.IntakeConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link IntakeSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The intake picks up fuel balls via a roller chain driven by a single NEO Vortex (1:1) and
 * captures them in a fabric bag void on the robot. A 2-motor lift (25:1 NEO × 2) raises or lowers
 * the entire intake assembly to satisfy the frame-perimeter size rule.
 *
 * <h2>Encoder Convention (Lift)</h2>
 * <ul>
 *   <li>Encoder = 0 → fully retracted (intake at retracted hardstop — homing reference)</li>
 *   <li>Encoder negative → intake lowered (match / collection position)</li>
 * </ul>
 *
 * <h2>Homing</h2>
 * <p>The intake is slowly raised until EITHER lift motor exceeds
 * {@link #homingCurrentThresholdAmps} (indicating retracted hardstop contact). Both encoders are
 * then zeroed. Homing can be re-triggered mid-match when encoders are suspected to have drifted.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code liftMotor1Inverted} / {@code liftMotor2Inverted} — both motors are mounted
 *       in the same physical orientation (not mirror-image), so both should spin in the same
 *       direction; confirm one inversion value covers both during first lift test</li>
 *   <li>{@code homingCurrentThresholdAmps} — tune by watching
 *       {@code Intake/Lift/Motor1/Current} and {@code Intake/Lift/Motor2/Current}</li>
 *   <li>{@code raisedPositionRotations} — measure on physical robot</li>
 *   <li>{@code raiseSpeed}, {@code lowerSpeed} — tune during first lift tests</li>
 *   <li>{@code intakeSpeed}, {@code reverseSpeed} — tune during first ball-pickup tests</li>
 * </ul>
 */
@Data
@Builder
public class IntakeSubsystemContext {

    /**
     * Creates a IntakeSubsystemContext with default values from {@link frc.robot.Constants.IntakeConstants}.
     *
     * @return Default configuration context
     */
    public static IntakeSubsystemContext defaults() {
        return IntakeSubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the roller motor controller (NEO Vortex on SparkFlex). */
    @Builder.Default
    private final int rollerMotorId = ROLLER_MOTOR_ID;

    /** CAN ID of lift motor 1 (NEO on SparkMax — inside the intake walls). */
    @Builder.Default
    private final int liftMotor1Id = LIFT_MOTOR_1_ID;

    /** CAN ID of lift motor 2 (NEO on SparkMax — outside the intake walls). */
    @Builder.Default
    private final int liftMotor2Id = LIFT_MOTOR_2_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for the roller motor in amps. */
    @Builder.Default
    private final int rollerCurrentLimit = ROLLER_CURRENT_LIMIT;

    /** Smart current limit for each lift motor in amps. */
    @Builder.Default
    private final int liftCurrentLimit = LIFT_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratio
    // -------------------------------------------------------------------------

    /** Gear reduction between each NEO shaft and the lift output. 25:1. */
    @Builder.Default
    private final double liftGearRatio = LIFT_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Motor inversion — confirm with build team
    // -------------------------------------------------------------------------

    /**
     * Whether to invert lift motor 1 (inside intake walls).
     * Both motors are mounted in the same physical orientation, so both should
     * spin in the same direction. If the lift runs backwards, flip this value
     * (and set motor 2 to match).
     * <b>TODO: confirm during first lift test.</b>
     */
    @Builder.Default
    private final boolean liftMotor1Inverted = false;

    /**
     * Whether to invert lift motor 2 (outside intake walls).
     * Same-direction mounting means this should match motor 1.
     * <b>TODO: confirm during first lift test.</b>
     */
    @Builder.Default
    private final boolean liftMotor2Inverted = false;

    // -------------------------------------------------------------------------
    // Roller speeds [-1.0, 1.0]
    // -------------------------------------------------------------------------

    /**
     * Roller speed for collecting balls. Positive = rollers spin inward.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double intakeSpeed = INTAKE_SPEED;

    /**
     * Roller speed for ejecting balls. Negative = rollers spin outward.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double reverseSpeed = REVERSE_SPEED;

    // -------------------------------------------------------------------------
    // Lift speeds [-1.0, 1.0]
    // Convention: positive = raise intake, negative = lower intake
    // -------------------------------------------------------------------------

    /**
     * Speed for raising intake toward stowed position. Positive.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double raiseSpeed = RAISE_SPEED;

    /**
     * Speed for lowering intake toward match position. Negative.
     * Kept slower than {@code raiseSpeed} since gravity assists the descent.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double lowerSpeed = LOWER_SPEED;

    /**
     * Speed for homing — slow upward creep toward retracted hardstop contact.
     * Positive. Kept slow to avoid hard impact.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double homingSpeed = HOMING_SPEED;

    // -------------------------------------------------------------------------
    // Homing — retracted-hardstop current detection
    // -------------------------------------------------------------------------

    /**
     * Current threshold (amps) signalling retracted hardstop contact during homing.
     * Homing stops when EITHER lift motor current exceeds this value.
     *
     * <p>Tune empirically: run homing, watch {@code Intake/Lift/Motor1/Current}
     * and {@code Intake/Lift/Motor2/Current}, note the spike at retracted hardstop contact,
     * set just below it.
     */
    @Builder.Default
    private final double homingCurrentThresholdAmps = HOMING_CURRENT_THRESHOLD_AMPS;

    // -------------------------------------------------------------------------
    // Encoder setpoints (motor rotations from homed zero)
    // Zero = fully retracted (hardstop contact). Negative = intake lowered.
    // -------------------------------------------------------------------------

    /**
     * Encoder position at fully-lowered (normal match) position.
     * Negative from homed zero. Intake rests on extended hardstop here.
     * <b>TODO: measure on physical robot.</b>
     */
    @Builder.Default
    private final double loweredPositionRotations = LOWERED_POSITION_ROTATIONS;

    /**
     * Encoder position at fully-retracted (stowed for climb) position.
     * Zero — established by homing. Intake rests on retracted hardstop here.
     */
    @Builder.Default
    private final double raisedPositionRotations = RAISED_POSITION_ROTATIONS;

    /**
     * Acceptable position error (rotations) for setpoint commands.
     * <b>TODO: tighten after physical testing.</b>
     */
    @Builder.Default
    private final double positionToleranceRotations = POSITION_TOLERANCE_ROTATIONS;
}
