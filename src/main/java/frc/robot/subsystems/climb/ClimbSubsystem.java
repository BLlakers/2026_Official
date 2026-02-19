package frc.robot.subsystems.climb;

import static java.util.Objects.requireNonNull;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * Climb subsystem for the 2-stage telescoping ratchet climb mechanism.
 *
 * <h2>Mechanism Overview</h2>
 * <p>A single NEO motor winds a cord on a spool to retract a telescoping arm, lifting the robot.
 * Two passive ratcheting hooks on each side of the arm catch rungs as the robot is lifted.
 * The telescope can extend <em>below</em> the robot frame when airborne, allowing it to reach
 * downward for the next rung.
 *
 * <h2>Climb Cycle (teleop)</h2>
 * <ol>
 *   <li>Arm extends downward to hook position ({@link #getExtendToHookPositionCommand})</li>
 *   <li>Arm retracts — passive hooks catch rung, robot lifts ({@link #getClimbNextRungCommand})</li>
 *   <li>Repeat for each rung until rung 3 (top). Hold until match end.</li>
 * </ol>
 *
 * <h2>Auto</h2>
 * <p>{@link #getRetractToAutoHeightCommand} lifts the robot just off the ground — hooks do not
 * need to engage. At teleop start, {@link #getLowerToGroundCommand} returns the robot to ground
 * so it can drive, followed by re-homing.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>Homing extends the telescope DOWN until it hits the mechanical hardstop (current spike).
 *       Encoder is zeroed at that point.</li>
 *   <li>Encoder ≈ 0 = telescope fully extended downward (hardstop)</li>
 *   <li>Encoder negative = telescope retracted (robot lifted)</li>
 *   <li>All lift setpoints ({@code rung1LiftRotations} etc.) are negative values.</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — positive output should extend telescope <strong>down</strong></li>
 *   <li>Tune {@code homingCurrentThresholdAmps} watching {@code Climb/Motor/Current} in Shuffleboard</li>
 *   <li>Measure {@code autoLiftRotations}, {@code rung1/2/3LiftRotations} during testing</li>
 *   <li>Confirm {@code extendedPositionRotations} and {@code positionToleranceRotations}</li>
 * </ul>
 */
public class ClimbSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Climb";

    /** Operating states of the climb mechanism. */
    public enum State {
        /** Motor stopped, encoder position unknown — safe before homing. */
        IDLE,
        /** Homing: slowly extending down to find the mechanical hardstop and zero the encoder. */
        HOMING,
        /** Telescope fully extended downward; hooks are in position for the next rung. */
        EXTENDED,
        /** Actively winding cord in to lift the robot (hooks engaged on a rung). */
        RETRACTING,
        /** Retraction complete; robot is lifted. Motor braking holds position. */
        HOLDING,
    }

    private final ClimbSubsystemContext context;
    private final SparkMax winchMotor;
    private final RelativeEncoder encoder;

    private State currentState = State.IDLE;

    /**
     * Internal counter tracking how many rungs have been successfully climbed in the current teleop
     * period. Resets to 0 on homing. Used by {@link #getClimbNextRungCommand()} to select the
     * appropriate setpoint.
     */
    private int currentRung = 0;

    /** The encoder target currently being sought by a position command. Used for telemetry. */
    private double targetRotations = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates a new ClimbSubsystem with default {@link ClimbSubsystemContext}.
     */
    public ClimbSubsystem() {
        this(ClimbSubsystemContext.defaults());
    }

    /**
     * Instantiates a new ClimbSubsystem with the specified context.
     *
     * @param context The ClimbSubsystemContext to apply to this instance
     */
    public ClimbSubsystem(final ClimbSubsystemContext context) {
        requireNonNull(context, "ClimbSubsystemContext cannot be null");
        this.context = context;

        this.winchMotor = new SparkMax(this.context.getMotorId(), MotorType.kBrushless);
        this.encoder = this.winchMotor.getEncoder();

        configureMotor();
        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    /**
     * Configures the winch motor controller with current limits, brake mode, and safe parameters.
     */
    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(this.context.getMotorCurrentLimit());
        config.idleMode(IdleMode.kBrake); // Brake mode holds arm position when motor is stopped
        // TODO: Set config.inverted(true/false) once motor direction is confirmed with build team.
        //       Convention: positive output = telescope extends DOWN, negative = retracts UP.
        this.winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // State & position helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    /**
     * Returns the current operating state of the climb mechanism.
     *
     * @return Current {@link State}
     */
    public State getState() {
        return currentState;
    }

    /**
     * Returns whether homing has been completed (encoder is zeroed and mechanism is safe to use).
     *
     * @return true if state is not IDLE or HOMING
     */
    public boolean isHomed() {
        return currentState != State.IDLE && currentState != State.HOMING;
    }

    /** Returns the number of rungs successfully climbed in this teleop period (0–3). */
    public int getCurrentRung() {
        return currentRung;
    }

    /** Returns true if the encoder is within tolerance of the given target. */
    private boolean atTarget(double targetRotations) {
        return Math.abs(encoder.getPosition() - targetRotations) <= this.context.getPositionToleranceRotations();
    }

    /** Returns the setpoint for the next rung in the sequence, or the last rung if already at top. */
    private double nextRungSetpoint() {
        return switch (currentRung) {
            case 0 -> context.getRung1LiftRotations();
            case 1 -> context.getRung2LiftRotations();
            default -> context.getRung3LiftRotations(); // rung 2 or beyond → target rung 3
        };
    }

    // -------------------------------------------------------------------------
    // Motor actions (private — exposed through command factories)
    // -------------------------------------------------------------------------

    /** Winds cord in, retracting the telescope upward to lift the robot. */
    private void retract() {
        setState(State.RETRACTING);
        winchMotor.set(this.context.getRetractSpeed());
    }

    /** Lets cord out, extending the telescope downward. */
    private void extend() {
        winchMotor.set(this.context.getExtendDownSpeed());
    }

    /** Stops the motor; brake mode holds current position. */
    private void hold() {
        setState(State.HOLDING);
        winchMotor.set(0);
    }

    /** Stops the motor (for safe shutdown / emergency stop). */
    private void stop() {
        setState(State.IDLE);
        winchMotor.set(0);
    }

    // -------------------------------------------------------------------------
    // Homing helpers
    // -------------------------------------------------------------------------

    /** Returns true when a current spike indicates the arm has hit its mechanical hardstop. */
    private boolean isAtHardstop() {
        return winchMotor.getOutputCurrent() >= this.context.getHomingCurrentThresholdAmps();
    }

    /** Zeros the encoder and resets rung counter after a successful homing. */
    private void completeHoming() {
        winchMotor.set(0);
        encoder.setPosition(0.0);
        currentRung = 0;
        targetRotations = 0.0;
        setState(State.EXTENDED);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Homing command — slowly extends the telescope downward until the arm hits its mechanical
     * hardstop (detected via motor current spike), then zeroes the encoder.
     *
     * <p>After homing: encoder = 0 = fully extended down (hardstop). Negative = retracted (lifted).
     * Resets the internal rung counter to 0.
     *
     * <p>This command is chained after {@link #getLowerToGroundCommand()} in
     * {@code scheduleTeleopInit()} — do not call it directly if the robot may be elevated.
     *
     * @return Command that homes the mechanism
     */
    public Command getHomingCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.HOMING);
                            winchMotor.set(this.context.getHomingSpeed());
                        },
                        () -> winchMotor.set(0))
                .until(this::isAtHardstop)
                .andThen(this.runOnce(this::completeHoming))
                .withName("Climb.Home");
    }

    /**
     * Lower-to-ground command — position-based: extends telescope down to
     * {@code extendedPositionRotations} (the hardstop / fully-extended position).
     *
     * <p>Used at teleop start after an auto climb to return the robot to the ground so it can drive.
     * If the robot is already at ground level the command completes immediately.
     *
     * @return Command that lowers the robot to ground
     */
    public Command getLowerToGroundCommand() {
        return this.run(() -> {
                    targetRotations = context.getExtendedPositionRotations();
                    if (!atTarget(context.getExtendedPositionRotations())) {
                        setState(State.RETRACTING); // extending down, semantically "lowering"
                        extend();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getExtendedPositionRotations()))
                .andThen(this.runOnce(() -> setState(State.EXTENDED)))
                .withName("Climb.LowerToGround");
    }

    /**
     * Extend-to-hook-position command — position-based: extends the telescope downward to
     * {@code extendedPositionRotations} so the passive hooks are in position for the next rung.
     *
     * <p>Auto-stops when at the target. Interruptible — manual extend/retract commands
     * will cancel it automatically if pressed.
     *
     * @return Command that extends the telescope to hook position
     */
    public Command getExtendToHookPositionCommand() {
        return this.run(() -> {
                    targetRotations = context.getExtendedPositionRotations();
                    if (!atTarget(context.getExtendedPositionRotations())) {
                        extend();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getExtendedPositionRotations()))
                .andThen(this.runOnce(() -> setState(State.EXTENDED)))
                .withName("Climb.ExtendToHookPosition");
    }

    /**
     * Auto-lift command — position-based: retracts the telescope to {@code autoLiftRotations},
     * just enough to lift the robot off the ground.
     *
     * <p>The passive hooks do NOT need to engage — this is the auto climb requirement only.
     * Must be followed by {@link #getLowerToGroundCommand()} at teleop start.
     *
     * @return Command that lifts the robot for auto
     */
    public Command getRetractToAutoHeightCommand() {
        return this.run(() -> {
                    targetRotations = context.getAutoLiftRotations();
                    if (!atTarget(context.getAutoLiftRotations())) {
                        retract();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getAutoLiftRotations()))
                .andThen(this.runOnce(this::hold))
                .withName("Climb.RetractToAutoHeight");
    }

    /**
     * Climb-next-rung command — the primary teleop climb command.
     *
     * <p>Retracts the telescope to the next rung setpoint in sequence, engaging the ratcheting hooks
     * and lifting the robot. Auto-stops when at the target. The internal rung counter advances only
     * on successful completion — if the driver interrupts the command (via manual override), the
     * counter does not advance and the command can be re-triggered.
     *
     * <p>Rung progression: 1 → 27", 2 → 45", 3 → 63". At rung 3 the robot holds until match end.
     *
     * <p>Interruptible — {@link #getManualRetractCommand()} and {@link #getManualExtendCommand()}
     * will cancel this command automatically when pressed.
     *
     * @return Command that climbs to the next rung in sequence
     */
    public Command getClimbNextRungCommand() {
        return this.run(() -> {
                    double setpoint = nextRungSetpoint();
                    targetRotations = setpoint;
                    if (!atTarget(setpoint)) {
                        retract();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(nextRungSetpoint()))
                .andThen(this.runOnce(() -> {
                    hold();
                    if (currentRung < 3) currentRung++;
                }))
                .withName("Climb.NextRung");
    }

    /**
     * Manual retract command — held-button override for retracting the telescope upward.
     * Holds position on release. Interrupts any running position-based command.
     *
     * @return Command that manually retracts the climb arm
     */
    public Command getManualRetractCommand() {
        return this.runEnd(this::retract, this::hold).withName("Climb.ManualRetract");
    }

    /**
     * Manual extend command — held-button override for extending the telescope downward.
     * Holds position on release. Interrupts any running position-based command.
     *
     * @return Command that manually extends the climb arm
     */
    public Command getManualExtendCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.EXTENDED);
                            extend();
                        },
                        this::hold)
                .withName("Climb.ManualExtend");
    }

    /**
     * Hold command — stops the motor and relies on brake mode to hold current position.
     *
     * @return Command that holds the current arm position
     */
    public Command getHoldCommand() {
        return this.runOnce(this::hold).withName("Climb.Hold");
    }

    /**
     * Stop command — stops the motor (for safe shutdown / emergency).
     *
     * @return Command that stops the climb mechanism
     */
    public Command getStopCommand() {
        return this.runOnce(this::stop).withName("Climb.Stop");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "MotorID=" + context.getMotorId());
        Telemetry.record(TELEMETRY_PREFIX + "/Config/MotorId", context.getMotorId(), TelemetryLevel.MATCH);
        Telemetry.record(TELEMETRY_PREFIX + "/Config/GearRatio", context.getGearRatio(), TelemetryLevel.MATCH);
    }

    private void captureTelemetry(String prefix) {
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/CurrentRung", currentRung, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Encoder/PositionRotations", encoder.getPosition(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Encoder/TargetRotations", targetRotations, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Encoder/AtTarget", atTarget(targetRotations) ? 1.0 : 0.0, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Motor/OutputPercent", winchMotor.getAppliedOutput(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Motor/Current", winchMotor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.record(
                prefix + "/Homing/CurrentThreshold", context.getHomingCurrentThresholdAmps(), TelemetryLevel.LAB);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Telemetry is captured by the registered subsystem callback via Telemetry.periodic()
    }
}
