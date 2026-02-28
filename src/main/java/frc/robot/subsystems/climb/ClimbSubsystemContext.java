package frc.robot.subsystems.climb;

import static frc.robot.Constants.ClimbConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link ClimbSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The climb mechanism is a 2-stage telescope driven by a single NEO motor on a SPARK MAX.
 * The second stage extends <em>upward</em> out of the first stage to reach bars above the robot.
 * A <strong>top hook/catch</strong> on the second stage grabs the bar. Retraction first nests the
 * stages, then pulls the entire telescope assembly upward through the robot frame until
 * <strong>passive ratcheting hooks</strong> (on the assembly) engage the bar.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>Encoder = 0 → stored: stages nested, assembly at lowest frame position (on the ground)</li>
 *   <li>Encoder positive → second stage extending upward (reaching for a bar)</li>
 *   <li>Encoder negative → assembly traveling through frame bottom (hooks rising toward bar)</li>
 * </ul>
 *
 * <p>There is <strong>no internal hardstop</strong> between the stages. The homing zero point is
 * established by the <strong>REV Through Bore Encoder</strong> on the spool shaft: the motor
 * retracts slowly until the encoder reads the calibrated stored angle, then the relative encoder
 * is zeroed. If the mechanism is already at the stored position at boot, homing is skipped
 * entirely. <strong>The robot must be on the ground for homing.</strong>
 *
 * <p>When hanging from a bar, the motor overcomes the first-stage spring and the relative encoder
 * freely goes negative as the assembly travels through the frame.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code gearRatio} — confirm from mechanism CAD</li>
 *   <li>{@code spoolCircumferenceMeters} — measure from spool diameter</li>
 *   <li>{@code throughBoreStoredAngleRotations} — calibrate: place in stored position,
 *       read {@code Climb/ThroughBore/RawAngle} in the Lab tab, enter the value here</li>
 *   <li>{@code bar1/2/3ExtendRotations} — measure how far to extend to reach each bar</li>
 *   <li>{@code bar1/2/3EngageRotations} — measure how far to retract for hooks to engage each bar</li>
 *   <li>{@code positionToleranceRotations} — how close is "close enough" to a setpoint</li>
 *   <li>Confirm motor inversion — positive output should extend telescope <strong>upward</strong></li>
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
     * Motor output [0, 1] for extending the telescope upward (reaching for a bar).
     * Should be positive (positive output = telescope extends up).
     */
    @Builder.Default
    private final double extendUpSpeed = EXTEND_UP_SPEED;

    /**
     * Motor output [-1, 0] for retracting the telescope (nesting stages / pulling through frame).
     * Should be negative.
     */
    @Builder.Default
    private final double retractSpeed = RETRACT_SPEED;

    /**
     * Slow motor output used during homing (retracts telescope toward ground hardstop).
     * Should be negative. Kept slower than {@code retractSpeed} to avoid excessive impact.
     * <b>Robot must be on the ground for homing — the ground provides the hardstop.</b>
     */
    @Builder.Default
    private final double homingSpeed = HOMING_SPEED;

    /**
     * Smart current limit for the winch motor in amps.
     */
    @Builder.Default
    private final int motorCurrentLimit = MOTOR_CURRENT_LIMIT;

    /**
     * Current threshold in amps that signals the telescope has reached the ground-contact hardstop.
     * There is no internal mechanical hardstop — the spike occurs when the stages are nested and
     * the ground prevents the assembly from traveling further through the frame.
     * <b>Tune empirically on a flat surface — watch {@code Climb/Motor/Current} in Shuffleboard.</b>
     */
    @Builder.Default
    private final double homingCurrentThresholdAmps = HOMING_CURRENT_THRESHOLD_AMPS;

    // -------------------------------------------------------------------------
    // Through-bore encoder (REV Through Bore Encoder on spool output shaft, DIO 5)
    // Provides absolute single-turn position — used for homing instead of current-spike
    // detection. If the mechanism is at the stored position at boot the subsystem
    // auto-seeds and skips the homing sequence entirely.
    // -------------------------------------------------------------------------

    /**
     * DIO channel for the REV Through Bore Encoder on the spool output shaft.
     * Defaults to {@link frc.robot.Constants.ClimbConstants#THROUGH_BORE_ENCODER_DIO_CHANNEL}.
     */
    @Builder.Default
    private final int throughBoreEncoderDioChannel = THROUGH_BORE_ENCODER_DIO_CHANNEL;

    /**
     * Absolute encoder angle [0, 1 rotation) that corresponds to the stored (zero) position.
     * Calibrate on the physical robot: place in stored position, read
     * {@code Climb/ThroughBore/RawAngle} in the Lab tab, enter the reading here and in
     * {@link frc.robot.Constants.ClimbConstants#THROUGH_BORE_STORED_ANGLE_ROTATIONS}.
     */
    @Builder.Default
    private final double throughBoreStoredAngleRotations = THROUGH_BORE_STORED_ANGLE_ROTATIONS;

    /**
     * Acceptable error (rotations) when comparing the through-bore reading to
     * {@link #throughBoreStoredAngleRotations}. Wrap-around near the 0/1 boundary is handled.
     * 0.02 rotations ≈ 7°.
     */
    @Builder.Default
    private final double throughBoreAngleTolerance = THROUGH_BORE_ANGLE_TOLERANCE_ROTATIONS;

    // -------------------------------------------------------------------------
    // Encoder setpoints (rotations from stored/0)
    //   positive = extended upward
    //   negative = assembly through frame (hooks engaging)
    // -------------------------------------------------------------------------

    /**
     * Encoder position at the stored/nested state on the ground.
     * This is the zero reference established by homing.
     */
    @Builder.Default
    private final double storedPositionRotations = STORED_POSITION_ROTATIONS;

    // --- Auto setpoints ---

    /**
     * Encoder position (positive) to extend to reach bar 1 during auto.
     * <b>Measure empirically.</b>
     */
    @Builder.Default
    private final double autoExtendRotations = AUTO_EXTEND_ROTATIONS;

    /**
     * Encoder position for the auto lift — just enough retraction to lift the robot off the
     * ground. May be positive (partial retraction from extension) or slightly negative.
     * Hooks do NOT need to engage.
     * <b>Measure empirically.</b>
     */
    @Builder.Default
    private final double autoEngageRotations = AUTO_ENGAGE_ROTATIONS;

    // --- Per-bar extend setpoints (positive — reaching upward) ---

    /**
     * Encoder position (positive) — extend upward to reach bar 1 from the ground.
     * This is the longest reach because ground-to-bar-1 distance &gt; bar-to-bar distance.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar1ExtendRotations = BAR_1_EXTEND_ROTATIONS;

    /**
     * Encoder position (positive) — extend upward to reach bar 2 from bar 1.
     * Shorter than bar 1 reach.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar2ExtendRotations = BAR_2_EXTEND_ROTATIONS;

    /**
     * Encoder position (positive) — extend upward to reach bar 3 from bar 2.
     * Similar to bar 2 reach.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar3ExtendRotations = BAR_3_EXTEND_ROTATIONS;

    // --- Per-bar engage setpoints (negative — assembly through frame, hooks catching) ---

    /**
     * Encoder position (negative) — retract through frame until passive hooks engage bar 1.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar1EngageRotations = BAR_1_ENGAGE_ROTATIONS;

    /**
     * Encoder position (negative) — retract through frame until passive hooks engage bar 2.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar2EngageRotations = BAR_2_ENGAGE_ROTATIONS;

    /**
     * Encoder position (negative) — retract through frame until passive hooks engage bar 3.
     * Robot holds here until end of match.
     * <b>Measure empirically during first climb tests.</b>
     */
    @Builder.Default
    private final double bar3EngageRotations = BAR_3_ENGAGE_ROTATIONS;

    /**
     * Acceptable position error in rotations when checking if a setpoint has been reached.
     * Larger values allow the command to complete earlier but with less precision.
     */
    @Builder.Default
    private final double positionToleranceRotations = POSITION_TOLERANCE_ROTATIONS;

    // -------------------------------------------------------------------------
    // AdvantageScope Pose3d visualization dimensions (meters)
    // -------------------------------------------------------------------------

    /**
     * Telescope visual length (meters) at full upward extension.
     * Based on 15" lower stage + 13.25" upper stage = 28.25" = 0.718 m.
     */
    @Builder.Default
    private final double maxTelescopeLength = MAX_TELESCOPE_LENGTH;

    /**
     * Minimum visual telescope length (meters) — stages fully nested.
     * Prevents the line from disappearing when the arm is fully retracted.
     */
    @Builder.Default
    private final double minTelescopeLength = MIN_TELESCOPE_LENGTH;

    /**
     * Horizontal portion of the passive side hook L-shape (meters).
     */
    @Builder.Default
    private final double sideHookHorizontalLength = SIDE_HOOK_HORIZONTAL_LENGTH;

    /**
     * Height of the passive hook mount on the telescope assembly (meters above ground when stored).
     * These hooks engage the bar when the assembly travels through the frame during retraction.
     * TODO: measure from CAD / physical robot (expected 6" or 9").
     */
    @Builder.Default
    private final double hookMountHeightMeters = HOOK_MOUNT_HEIGHT_METERS;

    /**
     * Lateral distance (meters) from robot center to each passive hook.
     * Left hook: robotX - offset; right hook: robotX + offset.
     * TODO: measure from CAD / physical robot.
     */
    @Builder.Default
    private final double hookOffsetMeters = HOOK_OFFSET_METERS;

    /**
     * Lateral distance (meters) from the robot center to the telescope arm (left-side mount).
     * Positive = left in robot frame. Robot approaches the tower left-side-first.
     * TODO: measure from CAD / physical robot.
     */
    @Builder.Default
    private final double telescopeSideOffsetMeters = TELESCOPE_SIDE_OFFSET_METERS;
}
