package frc.robot.subsystems.climb;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;
import static java.util.Objects.requireNonNull;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * Climb subsystem for the 2-stage telescoping ratchet climb mechanism.
 *
 * <h2>Mechanism Overview</h2>
 * <p>A single NEO motor winds a cord on a spool to control a 2-stage telescope. The second stage
 * extends <em>upward</em> out of the first stage, with a <strong>top hook/catch</strong> that grabs
 * a bar above the robot. Retraction first nests the stages, then pulls the entire telescope assembly
 * upward through the robot frame until <strong>passive ratcheting hooks</strong> (on the assembly)
 * engage the bar. The first-stage spring provides higher resistance during the through-frame phase.
 *
 * <h2>Climb Cycle (teleop)</h2>
 * <ol>
 *   <li>Arm extends upward — top hook reaches the next bar ({@link #getExtendToBarCommand})</li>
 *   <li>Arm retracts — stages nest, then assembly travels through frame, passive hooks catch the
 *       bar, robot lifts ({@link #getClimbNextBarCommand})</li>
 *   <li>Repeat for each bar until bar 3 (top). Hold until match end.</li>
 * </ol>
 *
 * <h2>Auto</h2>
 * <p>{@link #getRetractToAutoHeightCommand} extends the telescope to reach bar 1, then partially
 * retracts to lift the robot off the ground (hooks do not need to engage). At teleop start,
 * {@link #getLowerToGroundCommand} returns the robot to ground so it can drive, followed by
 * re-homing.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>Homing retracts the telescope until the <strong>REV Through Bore Encoder</strong> on the
 *       spool shaft reads the calibrated stored angle, then the relative encoder is zeroed.</li>
 *   <li>If the mechanism is at the stored position when the robot boots, homing is skipped
 *       (the subsystem auto-seeds from the absolute encoder in the constructor).</li>
 *   <li>Encoder = 0 = stored (stages nested, assembly at lowest frame position)</li>
 *   <li>Encoder positive = second stage extending upward (reaching for bar)</li>
 *   <li>Encoder negative = assembly traveling through frame bottom (hooks rising toward bar).
 *       Only possible when hanging — the motor overcomes the first-stage spring.</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — positive output should extend telescope <strong>upward</strong></li>
 *   <li>Calibrate {@code throughBoreStoredAngleRotations}: place in stored position, watch
 *       {@code Climb/ThroughBore/RawAngle} in the Lab tab, enter value in Constants</li>
 *   <li>Measure {@code bar1/2/3ExtendRotations} and {@code bar1/2/3EngageRotations} during testing</li>
 *   <li>Confirm {@code positionToleranceRotations}</li>
 * </ul>
 */
public class ClimbSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Climb";

    /** Operating states of the climb mechanism. */
    public enum State {
        /** Motor stopped, encoder position unknown — safe before homing. */
        IDLE,
        /** Homing: slowly retracting to find the ground-contact hardstop and zero the encoder. */
        HOMING,
        /** Telescope stored — stages nested, on the ground. Ready for first extend. */
        STORED,
        /** Telescope extending upward — second stage reaching for the next bar. */
        EXTENDING,
        /** Actively retracting to lift the robot (stages nesting, then assembly through frame). */
        RETRACTING,
        /** Retraction complete; robot is lifted. Motor braking holds position. */
        HOLDING,
    }

    private final ClimbSubsystemContext context;
    private final SparkMax winchMotor;
    private final RelativeEncoder encoder;

    /**
     * REV Through Bore Encoder on the spool output shaft (DIO 4).
     * Provides a single-turn absolute position used as a homing reference. Replaces the
     * current-spike detection — the homing command runs until this encoder reads the
     * calibrated stored angle rather than waiting for a current spike.
     */
    private final DutyCycleEncoder throughBoreEncoder;

    private final ClimbVisualizer visualizer;

    private State currentState = State.IDLE;

    /**
     * Internal counter tracking how many bars have been successfully climbed in the current teleop
     * period. Resets to 0 on homing. Used by {@link #getExtendToBarCommand()} and
     * {@link #getClimbNextBarCommand()} to select the appropriate setpoints.
     */
    private int currentBar = 0;

    /** The encoder target currently being sought by a position command. Used for telemetry. */
    private double targetRotations = 0.0;

    /** Motor dynamics engine for integrating position from applied voltage. */
    private DCMotorSim winchMotorSim;

    /** REV sim bridge — provides simulated motor current for homing detection. */
    private SparkMaxSim winchSparkMaxSim;

    /** Tracked encoder position in simulation (since the HAL bridge does not relay sim values). */
    private double simPosition = 0.0;

    /** Tracked motor current in simulation (since getOutputCurrent() returns 0 without sim). */
    private double simCurrent = 0.0;

    /** Timestamp of the last simulation tick, for computing dt. */
    private double lastSimTime = 0.0;

    /** Sim bridge for the through-bore encoder — drives the duty-cycle value in simulation. */
    private DutyCycleEncoderSim throughBoreEncoderSim;

    /**
     * Instantiates a new ClimbSubsystem with default context and no drivetrain reference.
     * AdvantageScope Pose3d publishing is disabled when no drivetrain is provided.
     */
    public ClimbSubsystem() {
        this(ClimbSubsystemContext.defaults(), null);
    }

    /**
     * Instantiates a new ClimbSubsystem with the specified context and no drivetrain reference.
     * AdvantageScope Pose3d publishing is disabled when no drivetrain is provided.
     *
     * @param context The ClimbSubsystemContext to apply to this instance
     */
    public ClimbSubsystem(final ClimbSubsystemContext context) {
        this(context, null);
    }

    /**
     * Instantiates a new ClimbSubsystem with the specified context and drivetrain reference.
     * The drivetrain is used to obtain the robot's field-relative pose for AdvantageScope
     * Pose3d visualization. Pass {@code null} to disable Pose3d publishing.
     *
     * @param context The ClimbSubsystemContext to apply to this instance
     * @param drivetrain The robot drivetrain, used for field-relative Pose3d coordinates
     */
    public ClimbSubsystem(final ClimbSubsystemContext context, final Drivetrain drivetrain) {
        requireNonNull(context, "ClimbSubsystemContext cannot be null");
        this.context = context;

        this.winchMotor = new SparkMax(this.context.getMotorId(), MotorType.kBrushless);
        this.encoder = this.winchMotor.getEncoder();
        this.throughBoreEncoder = new DutyCycleEncoder(this.context.getThroughBoreEncoderDioChannel());

        configureMotor();

        this.visualizer = new ClimbVisualizer(context, drivetrain);

        // Initialize simulation physics when running in sim
        if (RobotBase.isSimulation()) {
            this.winchSparkMaxSim = new SparkMaxSim(winchMotor, DCMotor.getNEO(1));
            // Build a state-space motor model: NEO motor, placeholder inertia, configured gear ratio.
            // The inertia value (0.01 kg·m²) is a sim-only placeholder — it controls how quickly the
            // motor accelerates in simulation but has no effect on real robot behavior.
            LinearSystem<N2, N1, N2> plant = createDCMotorSystem(DCMotor.getNEO(1), 0.01, context.getGearRatio());
            this.winchMotorSim = new DCMotorSim(plant, DCMotor.getNEO(1));
            this.lastSimTime = Timer.getFPGATimestamp();
            // Initialize through-bore encoder sim at the stored angle so homing completes
            // immediately in simulation (mechanism always starts stored in sim).
            this.throughBoreEncoderSim = new DutyCycleEncoderSim(this.throughBoreEncoder);
            this.throughBoreEncoderSim.set(this.context.getThroughBoreStoredAngleRotations());
        }

        // Boot-time absolute seeding: on a real robot the through-bore encoder always knows
        // the spool angle. If the mechanism is already at the stored position (normal at match
        // start), seed the relative encoder to 0 and skip the homing sequence entirely.
        if (RobotBase.isReal() && isAbsoluteAtStoredPosition()) {
            encoder.setPosition(0.0);
            currentState = State.STORED;
        }

        initializeTelemetry();
    }

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(this.context.getMotorCurrentLimit());
        config.idleMode(IdleMode.kBrake); // Brake mode holds arm position when motor is stopped
        // TODO: Set config.inverted() once motor direction is confirmed. Convention: positive = extend UP, negative =
        // retract.
        this.winchMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Returns the encoder position, routing through the simulation state when running in sim.
     * In simulation the HAL bridge does not relay {@link SparkMaxSim} values back to the real
     * {@link RelativeEncoder}, so we track position in {@link #simPosition} instead.
     */
    private double getEncoderPosition() {
        return RobotBase.isSimulation() ? simPosition : encoder.getPosition();
    }

    /**
     * Returns the motor output current, routing through the simulation state when running in sim.
     * In simulation the HAL does not provide current data for a raw {@link SparkMax}, so we use
     * the motor-model current computed by {@link SparkMaxSim} instead.
     */
    private double getMotorCurrent() {
        return RobotBase.isSimulation() ? simCurrent : winchMotor.getOutputCurrent();
    }

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

    /** Returns the number of bars successfully climbed in this teleop period (0–3). */
    public int getCurrentBar() {
        return currentBar;
    }

    /** Returns true if the encoder is within tolerance of the given target. */
    private boolean atTarget(double targetRotations) {
        return Math.abs(getEncoderPosition() - targetRotations) <= this.context.getPositionToleranceRotations();
    }

    /**
     * Returns the extend setpoint (positive) for the next bar in sequence.
     * Bar 1 extend is largest (ground to bar 1 is the longest reach).
     */
    private double nextExtendSetpoint() {
        return switch (currentBar) {
            case 0 -> context.getBar1ExtendRotations();
            case 1 -> context.getBar2ExtendRotations();
            default -> context.getBar3ExtendRotations(); // bar 2 or beyond → target bar 3
        };
    }

    /**
     * Returns the engage setpoint (negative) for the next bar in sequence.
     * The assembly must travel through the frame for passive hooks to catch the bar.
     */
    private double nextEngageSetpoint() {
        return switch (currentBar) {
            case 0 -> context.getBar1EngageRotations();
            case 1 -> context.getBar2EngageRotations();
            default -> context.getBar3EngageRotations();
        };
    }

    /** Lets cord out, extending the telescope upward (positive motor output). */
    private void extend() {
        winchMotor.set(this.context.getExtendUpSpeed());
    }

    /** Winds cord in, retracting the telescope (nesting stages / pulling through frame). */
    private void retract() {
        setState(State.RETRACTING);
        winchMotor.set(this.context.getRetractSpeed());
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

    /**
     * Returns true when a current spike indicates the telescope has reached the ground-contact
     * hardstop. There is no internal mechanical hardstop — the spike occurs when the stages are
     * nested and the first stage spring resistance pushes back.
     *
     * <p>Retained as a fallback diagnostic. The primary homing completion check is now
     * {@link #isAbsoluteAtStoredPosition()}, which uses the through-bore encoder.
     */
    private boolean isAtHardstop() {
        return getMotorCurrent() >= this.context.getHomingCurrentThresholdAmps();
    }

    /**
     * Returns true when the through-bore encoder reads within tolerance of the stored (zero)
     * position. This is the primary homing completion check — it replaces the current-spike
     * approach with a precise absolute angle check.
     *
     * <p>Wrap-around near the 0/1 boundary is handled: the difference is taken as the minimum
     * of the direct delta and its complement (1 − delta), so a stored angle of 0.02 and a
     * reading of 0.98 correctly produce a diff of 0.04 rather than 0.96.
     */
    private boolean isAbsoluteAtStoredPosition() {
        double raw = throughBoreEncoder.get(); // [0, 1)
        double stored = context.getThroughBoreStoredAngleRotations();
        double diff = Math.abs(raw - stored);
        double wrappedDiff = Math.min(diff, 1.0 - diff); // handle 0↔1 wrap-around
        return wrappedDiff <= context.getThroughBoreAngleTolerance();
    }

    /** Zeros the encoder and resets bar counter after a successful homing. */
    private void completeHoming() {
        winchMotor.set(0);
        encoder.setPosition(0.0);
        if (RobotBase.isSimulation()) {
            simPosition = 0.0;
            winchMotorSim.setState(VecBuilder.fill(0.0, 0.0));
        }
        currentBar = 0;
        targetRotations = 0.0;
        setState(State.STORED);
    }

    /**
     * Homing command — slowly retracts the telescope until the through-bore encoder reads the
     * calibrated stored angle ({@code throughBoreStoredAngleRotations}), then zeroes the relative
     * encoder.
     *
     * <p>After homing: encoder = 0 = stored (stages nested, assembly at lowest frame position).
     * Positive = extended upward; negative = assembly through frame.
     * Resets the internal bar counter to 0.
     *
     * <p><strong>The robot MUST be on the ground for homing.</strong> The through-bore encoder
     * reads the spool angle; the stored angle must be calibrated on the physical robot
     * (see {@code Climb/ThroughBore/RawAngle} in the Lab tab).
     * Do not call this while hanging.
     *
     * <p>If the mechanism was already at the stored position when the robot booted, this command
     * completes immediately (the subsystem was auto-seeded in the constructor).
     *
     * <p>This command is chained after {@link #getLowerToGroundCommand()} in
     * {@code scheduleTeleopInit()} to guarantee the robot is on the ground.
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
                .until(this::isAbsoluteAtStoredPosition)
                .andThen(this.runOnce(this::completeHoming))
                .withName("Climb.Home");
    }

    /**
     * Lower-to-ground command — extends the telescope (lets cord out) to lower the robot back
     * to the ground from a lifted position.
     *
     * <p>Target: {@code storedPositionRotations} (0.0). If the robot is already at ground level
     * the command completes immediately.
     *
     * <p>Used at teleop start after an auto climb to return the robot to the ground so it can
     * drive. <strong>Must precede homing</strong> — homing requires ground contact.
     *
     * @return Command that lowers the robot to ground
     */
    public Command getLowerToGroundCommand() {
        return this.run(() -> {
                    targetRotations = context.getStoredPositionRotations();
                    if (!atTarget(context.getStoredPositionRotations())) {
                        extend();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getStoredPositionRotations()))
                .andThen(this.runOnce(() -> setState(State.STORED)))
                .withName("Climb.LowerToGround");
    }

    /**
     * Extend-to-bar command — extends the telescope upward so the top hook reaches the next bar.
     *
     * <p>The extend target varies by bar (based on the internal bar counter):
     * <ul>
     *   <li>Bar 0 → bar 1: longest reach (ground-to-bar distance)</li>
     *   <li>Bar 1 → bar 2: shorter (bar-to-bar distance)</li>
     *   <li>Bar 2 → bar 3: similar to bar 2</li>
     * </ul>
     *
     * <p>Auto-stops when at the target. Does <strong>not</strong> advance the bar counter —
     * that happens in {@link #getClimbNextBarCommand()} after successful retraction.
     *
     * <p>Interruptible — manual extend/retract commands will cancel it automatically if pressed.
     *
     * @return Command that extends the telescope to the next bar
     */
    public Command getExtendToBarCommand() {
        return this.run(() -> {
                    double setpoint = nextExtendSetpoint();
                    targetRotations = setpoint;
                    if (!atTarget(setpoint)) {
                        setState(State.EXTENDING);
                        extend();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(nextExtendSetpoint()))
                .andThen(this.runOnce(() -> setState(State.HOLDING)))
                .withName("Climb.ExtendToBar");
    }

    /**
     * Climb-next-bar command — the primary teleop climb command.
     *
     * <p>Retracts the telescope to the engage setpoint for the current bar. The retraction first
     * nests the stages (encoder toward 0), then continues through the frame bottom (encoder goes
     * negative) until the passive hooks engage the bar.
     *
     * <p>Auto-stops when at the target. The internal bar counter advances only on successful
     * completion — if the driver interrupts (via manual override), the counter does not advance
     * and the command can be re-triggered.
     *
     * <p>Bar progression: 1 → 2 → 3. At bar 3 the robot holds until match end.
     *
     * <p>Interruptible — {@link #getManualRetractCommand()} and {@link #getManualExtendCommand()}
     * will cancel this command automatically when pressed.
     *
     * @return Command that climbs to the next bar in sequence
     */
    public Command getClimbNextBarCommand() {
        return this.run(() -> {
                    double setpoint = nextEngageSetpoint();
                    targetRotations = setpoint;
                    if (!atTarget(setpoint)) {
                        retract();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(nextEngageSetpoint()))
                .andThen(this.runOnce(() -> {
                    hold();
                    if (currentBar < 3) currentBar++;
                }))
                .withName("Climb.NextBar");
    }

    /**
     * Auto-lift command — two-step: extends the telescope to reach bar 1 ({@code autoExtendRotations}),
     * then partially retracts ({@code autoEngageRotations}) to lift the robot off the ground.
     *
     * <p>The passive hooks do NOT need to engage — this is the auto climb requirement only.
     * Must be followed by {@link #getLowerToGroundCommand()} at teleop start.
     *
     * @return Command that lifts the robot for auto
     */
    public Command getRetractToAutoHeightCommand() {
        // Step 1: extend to reach bar 1
        Command extendStep = this.run(() -> {
                    targetRotations = context.getAutoExtendRotations();
                    if (!atTarget(context.getAutoExtendRotations())) {
                        setState(State.EXTENDING);
                        extend();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getAutoExtendRotations()));

        // Step 2: partially retract to lift off ground
        Command retractStep = this.run(() -> {
                    targetRotations = context.getAutoEngageRotations();
                    if (!atTarget(context.getAutoEngageRotations())) {
                        retract();
                    } else {
                        hold();
                    }
                })
                .until(() -> atTarget(context.getAutoEngageRotations()))
                .andThen(this.runOnce(this::hold));

        return extendStep.andThen(retractStep).withName("Climb.AutoLift");
    }

    /**
     * Manual retract command — held-button override for retracting the telescope.
     * Holds position on release. Interrupts any running position-based command.
     *
     * @return Command that manually retracts the climb arm
     */
    public Command getManualRetractCommand() {
        return this.runEnd(this::retract, this::hold).withName("Climb.ManualRetract");
    }

    /**
     * Manual extend command — held-button override for extending the telescope upward.
     * Holds position on release. Interrupts any running position-based command.
     *
     * @return Command that manually extends the climb arm
     */
    public Command getManualExtendCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.EXTENDING);
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

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "MotorID=" + context.getMotorId());
        Telemetry.publish(TELEMETRY_PREFIX + "/Config/MotorId", context.getMotorId(), TelemetryLevel.MATCH);
        Telemetry.publish(TELEMETRY_PREFIX + "/Config/GearRatio", context.getGearRatio(), TelemetryLevel.MATCH);
    }

    private void captureTelemetry(String prefix) {
        double position = getEncoderPosition();
        visualizer.update(position, isHomed());
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/CurrentBar", currentBar, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/CurrentBar", currentBar, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Encoder/PositionRotations", position, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Encoder/TargetRotations", targetRotations, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Encoder/AtTarget", atTarget(targetRotations) ? 1.0 : 0.0, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Motor/OutputPercent", winchMotor.getAppliedOutput(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Motor/Current", getMotorCurrent(), TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/Homing/CurrentThreshold", context.getHomingCurrentThresholdAmps(), TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/ThroughBore/RawAngle", throughBoreEncoder.get(), TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/ThroughBore/AtStoredPosition", isAbsoluteAtStoredPosition() ? 1.0 : 0.0, TelemetryLevel.LAB);
    }

    @Override
    public void periodic() {}

    /**
     * Advances the simulated winch motor physics each tick when running in simulation.
     *
     * <p>This method is called automatically by the {@link edu.wpi.first.wpilibj2.command.CommandScheduler}
     * for every registered subsystem when {@link RobotBase#isSimulation()} is true.
     *
     * <h3>Physics model</h3>
     * <ol>
     *   <li>Read the duty-cycle output that commands have set via {@code winchMotor.set()}</li>
     *   <li>Convert to voltage and feed into a {@link DCMotorSim} (NEO motor model)</li>
     *   <li>Integrate position and velocity over dt</li>
     *   <li>Write results to {@link #simPosition} and {@link #simCurrent} so that
     *       {@link #getEncoderPosition()} and {@link #getMotorCurrent()} return correct values</li>
     * </ol>
     *
     * <h3>Ground hardstop simulation</h3>
     * <p>When the robot is on the ground (states: {@code HOMING}, {@code STORED}, {@code IDLE}),
     * the encoder position is clamped at 0 — simulating the ground blocking further retraction.
     * The through-bore encoder sim is initialized at the stored angle, so
     * {@link #isAbsoluteAtStoredPosition()} returns {@code true} immediately and the homing
     * command completes without needing to drive to the hardstop.
     *
     * <p>When hanging from a bar (states: {@code EXTENDING}, {@code RETRACTING}, {@code HOLDING}),
     * the ground stop is inactive and the encoder can freely go negative (assembly through frame).
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        // Applied motor output as voltage (duty cycle × battery voltage)
        double voltage = winchSparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();

        // Ground hardstop is active when the robot is on the ground (not hanging from a bar).
        // In these states, position cannot go below 0 — the ground blocks retraction.
        boolean groundStopActive =
                currentState == State.HOMING || currentState == State.STORED || currentState == State.IDLE;
        double currentPos = winchMotorSim.getAngularPositionRotations();

        if (groundStopActive && currentPos <= 0.0 && voltage < 0.0) {
            // At ground — clamp position at 0, velocity at 0. The motor stalls against the ground,
            // producing a high current that triggers homing detection.
            winchMotorSim.setState(VecBuilder.fill(0.0, 0.0));
            winchSparkMaxSim.iterate(0.0, RobotController.getBatteryVoltage(), dt);
            simPosition = 0.0;
        } else {
            // Free motion — feed voltage into the motor model and advance physics
            winchMotorSim.setInputVoltage(voltage);
            winchMotorSim.update(dt);
            winchSparkMaxSim.iterate(winchMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
            simPosition = winchMotorSim.getAngularPositionRotations();
        }

        // Update simulated current for homing detection via isAtHardstop()
        simCurrent = winchSparkMaxSim.getMotorCurrent();

        // Keep the through-bore encoder sim in sync with spool position.
        // The encoder wraps at 1.0 revolution — add the stored angle offset so that simPosition=0
        // maps to the configured stored angle, matching what the real encoder does.
        if (throughBoreEncoderSim != null) {
            double spoolRotations = simPosition / context.getGearRatio();
            double encoderAngle = (spoolRotations + context.getThroughBoreStoredAngleRotations()) % 1.0;
            if (encoderAngle < 0.0) encoderAngle += 1.0; // keep in [0, 1)
            throughBoreEncoderSim.set(encoderAngle);
        }
    }
}
