package frc.robot.subsystems.turret;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;
import static java.util.Objects.requireNonNull;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.function.DoubleSupplier;

/**
 * Turret subsystem — rotates the shooter assembly horizontally to aim at the hub.
 *
 * <h2>Mechanism Overview</h2>
 * <p>A single NEO drives a 20:1 gearbox whose output shaft drives an external ring gear /
 * pinion stage (ratio TBD from CAD). Together these rotate the shooter assembly horizontally
 * within a ±135° range of motion (270° total). The SparkMax built-in encoder tracks motor
 * rotations; angle at the turret is computed by dividing by the total gear ratio.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>0 rotations = home (turret aimed straight forward); established by homing</li>
 *   <li>Positive encoder count = counterclockwise (left) rotation</li>
 *   <li>Negative encoder count = clockwise (right) rotation</li>
 * </ul>
 * This matches the sign convention used by {@code TurretTracker}, so
 * {@code TurretTracker.getTurretAngleDegrees()} feeds directly into
 * {@link #getTrackCommand(DoubleSupplier)} without sign inversion.
 *
 * <h2>Gear Ratio Warning</h2>
 * <p>The context's {@code turretGearRatio} currently holds only the motor-side gearbox
 * (20:1). The full effective ratio must include the external ring gear / pinion stage.
 * <b>All encoder-based degree calculations are approximate until the full ratio is confirmed
 * from CAD and updated in {@link TurretSubsystemContext}.</b>
 *
 * <h2>Current Implementation Status (Stub)</h2>
 * <ul>
 *   <li>Manual jog commands work immediately for first-run testing</li>
 *   <li>{@link #getTrackCommand(DoubleSupplier)} is a proportional open-loop stub —
 *       replace with SparkMax closed-loop PID once gear ratio and encoder scaling are
 *       confirmed</li>
 *   <li>No homing routine yet — zero the encoder manually before first test</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — positive output must rotate turret counterclockwise (left)</li>
 *   <li>Update total gear ratio in {@link TurretSubsystemContext} from CAD ring gear data</li>
 *   <li>Tune proportional gain in {@link #getTrackCommand(DoubleSupplier)} or migrate to
 *       SparkMax closed-loop position control</li>
 *   <li>Add a homing routine (limit switch or current-spike detection at mechanical stop)</li>
 * </ul>
 */
public class TurretSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Turret";

    /**
     * Proportional gain for the open-loop tracking stub.
     * Maps degree error → motor output. Kept conservative.
     * <b>TODO: replace with SparkMax closed-loop PID after full gear ratio is confirmed.</b>
     */
    private static final double TRACKING_KP = 0.005; // TODO: tune

    /** Maximum motor output during tracking (clamps proportional output). */
    private static final double TRACKING_MAX_OUTPUT = 0.4;

    /** Operating states of the turret mechanism. */
    public enum State {
        /** Motor stopped, holding position (brake). */
        IDLE,
        /** Manual jog — rotating left (counterclockwise). */
        JOGGING_LEFT,
        /** Manual jog — rotating right (clockwise). */
        JOGGING_RIGHT,
        /**
         * Actively tracking a target angle from {@link frc.robot.subsystems.turrettracker.TurretTracker}.
         * Open-loop proportional stub — replace with closed-loop PID for production.
         */
        TRACKING,
    }

    private final TurretSubsystemContext context;

    // Motor
    private final SparkMax turretMotor; // NEO

    private State currentState = State.IDLE;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim turretMotorSim;
    private SparkMaxSim turretSparkMaxSim;
    private double lastSimTime = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates a TurretSubsystem with default context.
     */
    public TurretSubsystem() {
        this(TurretSubsystemContext.defaults());
    }

    /**
     * Instantiates a TurretSubsystem with the specified context.
     *
     * @param context The TurretSubsystemContext to apply to this instance
     */
    public TurretSubsystem(final TurretSubsystemContext context) {
        requireNonNull(context, "TurretSubsystemContext cannot be null");
        this.context = context;

        this.turretMotor = new SparkMax(context.getTurretMotorId(), MotorType.kBrushless);

        configureMotor();

        if (RobotBase.isSimulation()) {
            this.turretSparkMaxSim = new SparkMaxSim(turretMotor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> plant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.005, context.getTurretGearRatio());
            this.turretMotorSim = new DCMotorSim(plant, DCMotor.getNEO(1));
            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(context.getTurretCurrentLimit());
        config.idleMode(IdleMode.kBrake); // Brake — holds turret angle when motor stops
        config.inverted(context.isTurretMotorInverted());
        turretMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // State helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    /**
     * Returns the current operating state of the turret.
     *
     * @return Current {@link State}
     */
    public State getState() {
        return currentState;
    }

    // -------------------------------------------------------------------------
    // Encoder / position helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the current turret angle in degrees from home.
     *
     * <p>Computed as: {@code motorRotations / gearRatio × 360}.
     *
     * <p><b>Warning:</b> this value is only accurate when {@code turretGearRatio} in the
     * context reflects the <em>full</em> effective ratio (gearbox × ring gear stage).
     * Until confirmed from CAD, treat this as an approximation.
     *
     * @return Turret angle in degrees (positive = CCW/left, negative = CW/right)
     */
    public double getCurrentAngleDegrees() {
        return turretMotor.getEncoder().getPosition() / context.getTurretGearRatio() * 360.0;
    }

    /**
     * Returns true if the turret is within tolerance of the given target angle.
     *
     * @param targetDegrees Target turret angle in degrees
     * @return True if {@code |error| <= turretPositionToleranceDegrees}
     */
    public boolean isOnTarget(double targetDegrees) {
        return Math.abs(targetDegrees - getCurrentAngleDegrees()) <= context.getTurretPositionToleranceDegrees();
    }

    // -------------------------------------------------------------------------
    // Motor actions (private — exposed through command factories)
    // -------------------------------------------------------------------------

    private void jogLeft() {
        turretMotor.set(context.getTurretJogSpeed());
    }

    private void jogRight() {
        turretMotor.set(-context.getTurretJogSpeed());
    }

    private void stop() {
        turretMotor.set(0);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Jog-left command — slowly rotates the turret counterclockwise for bring-up testing.
     *
     * <p>Held-button: rotates while held, stops (holds brake) on release.
     * Intended for debug controller use only — not for normal match operation.
     *
     * @return Command that jogs the turret left while held
     */
    public Command getJogLeftCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.JOGGING_LEFT);
                            jogLeft();
                        },
                        () -> {
                            stop();
                            if (currentState == State.JOGGING_LEFT) setState(State.IDLE);
                        })
                .withName("Turret.JogLeft");
    }

    /**
     * Jog-right command — slowly rotates the turret clockwise for bring-up testing.
     *
     * <p>Held-button: rotates while held, stops (holds brake) on release.
     * Intended for debug controller use only — not for normal match operation.
     *
     * @return Command that jogs the turret right while held
     */
    public Command getJogRightCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.JOGGING_RIGHT);
                            jogRight();
                        },
                        () -> {
                            stop();
                            if (currentState == State.JOGGING_RIGHT) setState(State.IDLE);
                        })
                .withName("Turret.JogRight");
    }

    /**
     * Stop command — stops the turret motor (brake mode holds position).
     *
     * @return Command that stops the turret
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {
                    stop();
                    setState(State.IDLE);
                })
                .withName("Turret.Stop");
    }

    /**
     * Track command — continuously drives the turret toward the angle provided by the supplier.
     *
     * <p>Intended to run as the default command when {@link frc.robot.subsystems.turrettracker.TurretTracker}
     * is enabled. Wire it in {@code RobotContainer} as:
     * <pre>
     *   turretSubsystem.setDefaultCommand(
     *       turretSubsystem.getTrackCommand(turretTracker::getTurretAngleDegrees));
     * </pre>
     *
     * <p><b>Current implementation:</b> open-loop proportional control
     * ({@code output = Kp × error}), clamped to {@code TRACKING_MAX_OUTPUT}.
     * This is sufficient for initial validation but will not hold a precise angle under load.
     *
     * <p><b>TODO:</b> Replace with SparkMax closed-loop position control
     * ({@code SparkClosedLoopController} with position PID) once the full gear ratio is
     * confirmed and encoder scaling is validated on the physical robot.
     *
     * @param targetAngleDegreesSupplier Supplier of the desired turret angle in degrees
     *     (positive = CCW/left, negative = CW/right). Typically
     *     {@code TurretTracker::getTurretAngleDegrees}.
     * @return Command that continuously tracks the supplied angle
     */
    public Command getTrackCommand(DoubleSupplier targetAngleDegreesSupplier) {
        return this.run(() -> {
                    setState(State.TRACKING);
                    double targetDegrees = targetAngleDegreesSupplier.getAsDouble();
                    double errorDegrees = targetDegrees - getCurrentAngleDegrees();
                    double output = MathUtil.clamp(TRACKING_KP * errorDegrees, -TRACKING_MAX_OUTPUT, TRACKING_MAX_OUTPUT);
                    turretMotor.set(output);
                })
                .withName("Turret.Track");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "TurretMotorID=" + context.getTurretMotorId());
    }

    private void captureTelemetry(String prefix) {
        double angleDegrees = getCurrentAngleDegrees();

        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/AngleDeg", angleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/AngleDeg", angleDegrees, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/OutputPercent", turretMotor.getAppliedOutput(), TelemetryLevel.MATCH);

        // LAB level
        Telemetry.record(prefix + "/Current", turretMotor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/EncoderRotations", turretMotor.getEncoder().getPosition(), TelemetryLevel.LAB);

        // VERBOSE level
        if (RobotBase.isSimulation()) {
            Telemetry.record(prefix + "/VelocityRPM", turretMotorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
        }
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Telemetry is captured by the registered subsystem callback via Telemetry.periodic()
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    /**
     * Advances simulated turret motor physics each tick.
     *
     * <p>The moment of inertia is set slightly higher than the flywheel sims (0.005 vs 0.001)
     * to approximate the rotational inertia of the shooter assembly. No hardstop simulation
     * is included in the stub — the turret range limits will be enforced by the control
     * logic once closed-loop tracking is implemented.
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        double voltage = turretSparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        turretMotorSim.setInputVoltage(voltage);
        turretMotorSim.update(dt);
        turretSparkMaxSim.iterate(turretMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
    }
}
