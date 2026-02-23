package frc.robot.subsystems.hopper;

import static frc.robot.Constants.HopperConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link HopperSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The hopper picks up fuel balls via a roller chain driven by a single NEO Vortex (1:1) and
 * captures them in a fabric bag void on the robot. A 2-motor lift (25:1 NEO × 2) raises or lowers
 * the entire hopper assembly to satisfy the frame-perimeter size rule.
 *
 * <h2>Encoder Convention (Lift)</h2>
 * <ul>
 *   <li>Encoder = 0 → fully lowered (hopper resting on bumpers — homing reference)</li>
 *   <li>Encoder positive → hopper raised (stowed for climb / frame-perimeter compliance)</li>
 * </ul>
 *
 * <h2>Homing</h2>
 * <p>The hopper is slowly lowered until EITHER lift motor exceeds
 * {@link #homingCurrentThresholdAmps} (indicating bumper contact). Both encoders are then zeroed.
 * Homing can be re-triggered mid-match when encoders are suspected to have drifted.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code liftMotor1Inverted} / {@code liftMotor2Inverted} — both motors are mounted
 *       in the same physical orientation (not mirror-image), so both should spin in the same
 *       direction; confirm one inversion value covers both during first lift test</li>
 *   <li>{@code homingCurrentThresholdAmps} — tune by watching
 *       {@code Hopper/Lift/Motor1/Current} and {@code Hopper/Lift/Motor2/Current}</li>
 *   <li>{@code raisedPositionRotations} — measure on physical robot</li>
 *   <li>{@code raiseSpeed}, {@code lowerSpeed} — tune during first lift tests</li>
 *   <li>{@code intakeSpeed}, {@code reverseSpeed} — tune during first ball-pickup tests</li>
 * </ul>
 */
@Data
@Builder
public class HopperSubsystemContext {

    /**
     * Creates a HopperSubsystemContext with default values from {@link frc.robot.Constants.HopperConstants}.
     *
     * @return Default configuration context
     */
    public static HopperSubsystemContext defaults() {
        return HopperSubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the roller motor controller (NEO Vortex on SparkFlex). */
    @Builder.Default
    private final int rollerMotorId = ROLLER_MOTOR_ID;

    /** CAN ID of lift motor 1 (NEO on SparkMax — inside the hopper walls). */
    @Builder.Default
    private final int liftMotor1Id = LIFT_MOTOR_1_ID;

    /** CAN ID of lift motor 2 (NEO on SparkMax — outside the hopper walls). */
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
     * Whether to invert lift motor 1 (inside hopper walls).
     * Both motors are mounted in the same physical orientation, so both should
     * spin in the same direction. If the lift runs backwards, flip this value
     * (and set motor 2 to match).
     * <b>TODO: confirm during first lift test.</b>
     */
    @Builder.Default
    private final boolean liftMotor1Inverted = false;

    /**
     * Whether to invert lift motor 2 (outside hopper walls).
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
    // Convention: positive = raise hopper, negative = lower hopper
    // -------------------------------------------------------------------------

    /**
     * Speed for raising hopper toward stowed position. Positive.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double raiseSpeed = RAISE_SPEED;

    /**
     * Speed for lowering hopper toward match position. Negative.
     * Kept slower than {@code raiseSpeed} since gravity assists the descent.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double lowerSpeed = LOWER_SPEED;

    /**
     * Speed for homing — slow downward creep toward bumper contact.
     * Negative. Kept slow to avoid hard impact with bumpers.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double homingSpeed = HOMING_SPEED;

    // -------------------------------------------------------------------------
    // Homing — bumper-contact current detection
    // -------------------------------------------------------------------------

    /**
     * Current threshold (amps) signalling bumper contact during homing.
     * Homing stops when EITHER lift motor current exceeds this value.
     *
     * <p>Tune empirically: run homing, watch {@code Hopper/Lift/Motor1/Current}
     * and {@code Hopper/Lift/Motor2/Current}, note the spike at bumper contact,
     * set just below it.
     */
    @Builder.Default
    private final double homingCurrentThresholdAmps = HOMING_CURRENT_THRESHOLD_AMPS;

    // -------------------------------------------------------------------------
    // Encoder setpoints (motor rotations from homed zero)
    // Zero = fully lowered (bumper contact). Positive = hopper raised.
    // -------------------------------------------------------------------------

    /**
     * Encoder position at fully-lowered (normal match) position.
     * Zero — established by homing. Hopper rests on bumpers here.
     */
    @Builder.Default
    private final double loweredPositionRotations = LOWERED_POSITION_ROTATIONS;

    /**
     * Encoder position at fully-raised (stowed for climb) position.
     * <b>TODO: measure on physical robot.</b>
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
