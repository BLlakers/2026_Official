package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link ClimbSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <p>The climb mechanism is a telescoping arm driven by a single NEO motor on a SPARK MAX.
 * The motor winds a cord on a spool to retract the arm and lift the robot via passive ratcheting
 * hooks. Homing uses current spike detection to detect the mechanical hardstop at full extension —
 * no limit switch needed.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>Encoder = 0 → telescope fully extended downward (hardstop, hook position)</li>
 *   <li>Encoder negative → telescope retracted (robot lifted)</li>
 *   <li>All lift setpoints are negative values</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code gearRatio} — confirm from mechanism CAD</li>
 *   <li>{@code spoolCircumferenceMeters} — measure from spool diameter</li>
 *   <li>{@code homingCurrentThresholdAmps} — tune by watching {@code Climb/Motor/Current}
 *       in Shuffleboard during a slow homing run; set just below the spike at hardstop</li>
 *   <li>{@code autoLiftRotations} — measure: just enough to clear the ground</li>
 *   <li>{@code rung1/2/3LiftRotations} — measure empirically during first climb tests</li>
 *   <li>{@code positionToleranceRotations} — how close is "close enough" to a setpoint</li>
 *   <li>Confirm motor inversion — positive output should extend telescope <strong>down</strong></li>
 * </ul>
 */
@Data
@Builder
public class ClimbSubsystemContext {

    /**
     * Creates a ClimbSubsystemContext with default values.
     *
     * @return Default configuration context
     */
    public static ClimbSubsystemContext defaults() {
        return ClimbSubsystemContext.builder().build();
    }

    /**
     * CAN ID of the winch motor controller (NEO on SPARK MAX).
     */
    @Builder.Default
    private final int motorId = MOTOR_ID;

    /**
     * Gear ratio between the motor shaft and the spool output shaft.
     * Motor rotations = spool rotations × gearRatio.
     * <b>Placeholder — confirm from CAD.</b>
     */
    @Builder.Default
    private final double gearRatio = GEAR_RATIO;

    /**
     * Circumference of the cord spool in meters (π × diameter).
     * Used to convert motor rotations → cord length.
     * <b>Placeholder — measure from spool.</b>
     */
    @Builder.Default
    private final double spoolCircumferenceMeters = SPOOL_CIRCUMFERENCE_METERS;

    /**
     * Motor output [-1, 1] for retracting the arm (winding cord in / lifting robot).
     * Should be negative if positive output = telescope extends down.
     */
    @Builder.Default
    private final double retractSpeed = RETRACT_SPEED;

    /**
     * Motor output for extending the telescope downward (letting cord out).
     * Should be positive if positive output = telescope extends down.
     */
    @Builder.Default
    private final double extendDownSpeed = EXTEND_DOWN_SPEED;

    /**
     * Slow motor output used during homing (extends telescope down toward hardstop).
     * Kept separate from {@code extendDownSpeed} so it can be tuned more conservatively.
     */
    @Builder.Default
    private final double homingSpeed = HOMING_SPEED;

    /**
     * Smart current limit for the winch motor in amps.
     */
    @Builder.Default
    private final int motorCurrentLimit = MOTOR_CURRENT_LIMIT;

    /**
     * Current threshold in amps that signals the telescope has hit its mechanical hardstop.
     * When {@code winchMotor.getOutputCurrent()} exceeds this value during homing, the encoder
     * is zeroed.
     * <b>Tune empirically — watch {@code Climb/Motor/Current} in Shuffleboard.</b>
     */
    @Builder.Default
    private final double homingCurrentThresholdAmps = HOMING_CURRENT_THRESHOLD_AMPS;

    /**
     * Encoder position (rotations) of the telescope at full extension downward (the hardstop).
     * This is the zero reference point and the hook-engagement position.
     * Typically 0.0 since the encoder is zeroed here after homing.
     */
    @Builder.Default
    private final double extendedPositionRotations = EXTENDED_POSITION_ROTATIONS;

    /**
     * Encoder position (rotations, negative) at which the robot is just off the ground.
     * Used for the auto climb — hooks do not need to engage.
     * <b>Measure empirically.</b>
     */
    @Builder.Default
    private final double autoLiftRotations = AUTO_LIFT_ROTATIONS;

    /**
     * Encoder position (rotations, negative) at which the robot is fully lifted to rung 1 (27").
     * Hooks must be engaged on rung 1.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double rung1LiftRotations = RUNG_1_LIFT_ROTATIONS;

    /**
     * Encoder position (rotations, negative) at which the robot is fully lifted to rung 2 (45").
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double rung2LiftRotations = RUNG_2_LIFT_ROTATIONS;

    /**
     * Encoder position (rotations, negative) at which the robot is fully lifted to rung 3 (63").
     * The robot holds here until the end of the match.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double rung3LiftRotations = RUNG_3_LIFT_ROTATIONS;

    /**
     * Acceptable position error in rotations when checking if a setpoint has been reached.
     * Larger values allow the command to complete earlier but with less precision.
     */
    @Builder.Default
    private final double positionToleranceRotations = POSITION_TOLERANCE_ROTATIONS;
}
