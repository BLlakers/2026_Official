package frc.robot.subsystems.shooter;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;
import static java.util.Objects.requireNonNull;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.function.DoubleSupplier;

/**
 * Shooter subsystem — receives fuel balls from the indexer and fires them into the hub.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The shooter is a differential-velocity dual-roller launcher. A fixed-angle channel
 * (~23°) guides each ball past two independently controlled flywheels:
 * <ul>
 *   <li><b>Front flywheel (A)</b> — 3.0" diameter, contacts one side of the ball</li>
 *   <li><b>Rear flywheel (B)</b> — 4.0" diameter, contacts the opposite side</li>
 * </ul>
 * Both wheels spin in the same physical direction relative to the ball to eject it; because
 * they grip opposite sides, exactly one motor must be inverted so that positive output
 * on both motors fires the ball forward.
 *
 * <h2>Physics</h2>
 * <p>Exit velocity and backspin are both determined by the flywheel surface speeds:
 * <pre>
 *   v_exit = eta * (v_A + v_B) / 2        (controls range)
 *   omega  = eta_spin * (v_A - v_B) / d   (controls Magnus lift)
 * </pre>
 * where {@code v_A} and {@code v_B} are the surface speeds (m/s) of the front and rear
 * flywheels, {@code eta} is the energy transfer efficiency, and {@code d} is the ball
 * diameter. More backspin → more Magnus lift → higher, more arched trajectory.
 *
 * <p>The two flywheels are independently commanded to exploit the 4"/3" diameter ratio
 * (1.33× surface speed difference at equal RPM) for backspin control. The open-loop
 * speed stubs here are temporary — see {@code SHOOTER.md} for the calibration plan and
 * physics-based inverse solver that will replace them.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → ball fired toward the target (hub)</li>
 *   <li>Negative output → reverse to clear jams or eject balls back toward indexer</li>
 * </ul>
 *
 * <h2>Coordination</h2>
 * <p>The shooter is commanded in unison with the relay and indexer via a
 * {@code Commands.parallel()} group bound to the manipulator right trigger. It is not
 * intended to run independently during normal match play.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — exactly one motor must be inverted so that positive
 *       output on both motors fires the ball forward.
 *       See {@link ShooterSubsystemContext#isShooterFrontMotorInverted()} and
 *       {@link ShooterSubsystemContext#isShooterRearMotorInverted()}</li>
 *   <li>Tune open-loop speeds as an initial smoke-test before calibration</li>
 *   <li>After SHOOTER.md calibration sessions, replace percent-output control with
 *       PID velocity closed-loop and the physics-based inverse solver</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Shooter";

    /**
     * Step size applied to a flywheel speed setpoint on each increase/decrease command tap.
     * 0.02 = 2% motor output per tap → 50 steps across the full [0, 1] range.
     */
    private static final double SPEED_STEP = 0.02;

    /** Operating states of the shooter mechanism. */
    public enum State {
        /** Both flywheel motors stopped. */
        IDLE,
        /** Both flywheel motors spinning to fire balls. */
        SHOOTING,
        /** Both flywheel motors spinning in reverse to clear jams / eject balls. */
        REVERSING,
    }

    private final ShooterSubsystemContext context;

    // Motors — one per flywheel
    private final SparkMax frontMotor; // NEO — front flywheel A, 3" diameter
    private final SparkMax rearMotor; // NEO — rear flywheel B, 4" diameter

    // Runtime-adjustable speed setpoints — initialised from context, tunable via debug commands.
    // runForward() reads these each loop so changes take effect immediately while shooting.
    private double frontSpeedSetpoint;
    private double rearSpeedSetpoint;

    // Closed-loop control fields (feedforward + PID)
    private SimpleMotorFeedforward frontFeedforward;
    private SimpleMotorFeedforward rearFeedforward;
    private final PIDController frontPid;
    private final PIDController rearPid;

    // NetworkTable entries for live tuning via Shuffleboard
    private final NetworkTable shooterTable;
    private final NetworkTableEntry kPEntry;
    private final NetworkTableEntry kIEntry;
    private final NetworkTableEntry kDEntry;
    private final NetworkTableEntry kSEntry;
    private final NetworkTableEntry kVEntry;
    private final NetworkTableEntry kAEntry;
    private final NetworkTableEntry targetRPMEntry;

    // Target RPMs for closed-loop control
    private double targetFrontRPM = 0.0;
    private double targetRearRPM = 0.0;

    // Whether closed-loop velocity control is active
    private boolean closedLoopEnabled = false;

    private State currentState = State.IDLE;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim frontMotorSim;
    private SparkMaxSim frontSparkMaxSim;
    private DCMotorSim rearMotorSim;
    private SparkMaxSim rearSparkMaxSim;
    private double lastSimTime = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates a ShooterSubsystem with default context.
     */
    public ShooterSubsystem() {
        this(ShooterSubsystemContext.defaults());
    }

    /**
     * Instantiates a ShooterSubsystem with the specified context.
     *
     * @param context The ShooterSubsystemContext to apply to this instance
     */
    public ShooterSubsystem(final ShooterSubsystemContext context) {
        requireNonNull(context, "ShooterSubsystemContext cannot be null");
        this.context = context;

        this.frontMotor = new SparkMax(context.getShooterFrontMotorId(), MotorType.kBrushless);
        this.rearMotor = new SparkMax(context.getShooterRearMotorId(), MotorType.kBrushless);

        this.frontSpeedSetpoint = context.getShooterFrontSpeed();
        this.rearSpeedSetpoint = context.getShooterRearSpeed();

        configureMotors();

        // Initialize feedforward and PID controllers with constants from Constants.ShooterConstants
        // Create PID controllers (gains will be updated from Shuffleboard entries at runtime)
        this.frontPid = new PIDController(
                frc.robot.Constants.ShooterConstants.SHOOTER_kP,
                frc.robot.Constants.ShooterConstants.SHOOTER_kI,
                frc.robot.Constants.ShooterConstants.SHOOTER_kD);
        this.rearPid = new PIDController(
                frc.robot.Constants.ShooterConstants.SHOOTER_kP,
                frc.robot.Constants.ShooterConstants.SHOOTER_kI,
                frc.robot.Constants.ShooterConstants.SHOOTER_kD);

        // NetworkTables / Shuffleboard tuning table
        this.shooterTable = NetworkTableInstance.getDefault().getTable("Shooter");
        this.kPEntry = shooterTable.getEntry("kP");
        this.kIEntry = shooterTable.getEntry("kI");
        this.kDEntry = shooterTable.getEntry("kD");
        this.kSEntry = shooterTable.getEntry("kS");
        this.kVEntry = shooterTable.getEntry("kV");
        this.kAEntry = shooterTable.getEntry("kA");
        this.targetRPMEntry = shooterTable.getEntry("TargetRPM");

        // Initialize entries with defaults from Constants (setDefault ensures Shuffleboard doesn't overwrite existing
        // values)
        this.kPEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kP);
        this.kIEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kI);
        this.kDEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kD);
        this.kSEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KS);
        this.kVEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KV);
        this.kAEntry.setDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KA);
        this.targetRPMEntry.setDouble(3000.0);

        // Initialize feedforward objects from constants (they will be refreshed from NT in periodic())
        this.frontFeedforward = new SimpleMotorFeedforward(
                frc.robot.Constants.ShooterConstants.SHOOTER_KS,
                frc.robot.Constants.ShooterConstants.SHOOTER_KV,
                frc.robot.Constants.ShooterConstants.SHOOTER_KA);
        this.rearFeedforward = new SimpleMotorFeedforward(
                frc.robot.Constants.ShooterConstants.SHOOTER_KS,
                frc.robot.Constants.ShooterConstants.SHOOTER_KV,
                frc.robot.Constants.ShooterConstants.SHOOTER_KA);

        if (RobotBase.isSimulation()) {
            this.frontSparkMaxSim = new SparkMaxSim(frontMotor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> frontPlant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.001, context.getShooterFrontGearRatio());
            this.frontMotorSim = new DCMotorSim(frontPlant, DCMotor.getNEO(1));

            this.rearSparkMaxSim = new SparkMaxSim(rearMotor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> rearPlant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.001, context.getShooterRearGearRatio());
            this.rearMotorSim = new DCMotorSim(rearPlant, DCMotor.getNEO(1));

            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureMotors() {
        // Front flywheel (A, 3")
        SparkMaxConfig frontConfig = new SparkMaxConfig();
        frontConfig.smartCurrentLimit(context.getShooterCurrentLimit());
        frontConfig.idleMode(IdleMode.kCoast); // Coast — let flywheels spin down freely
        frontConfig.inverted(context.isShooterFrontMotorInverted());
        frontMotor.configure(frontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Rear flywheel (B, 4")
        SparkMaxConfig rearConfig = new SparkMaxConfig();
        rearConfig.smartCurrentLimit(context.getShooterCurrentLimit());
        rearConfig.idleMode(IdleMode.kCoast); // Coast — let flywheels spin down freely
        rearConfig.inverted(context.isShooterRearMotorInverted());
        rearMotor.configure(rearConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // State helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    /**
     * Returns the current operating state of the shooter.
     *
     * @return Current {@link State}
     */
    public State getState() {
        return currentState;
    }

    // -------------------------------------------------------------------------
    // Motor actions (private — exposed through command factories)
    // -------------------------------------------------------------------------

    private void runForward() {
        frontMotor.set(frontSpeedSetpoint);
        rearMotor.set(rearSpeedSetpoint);
    }

    private void adjustFrontSpeed(double delta) {
        frontSpeedSetpoint = Math.min(1.0, Math.max(0.0, frontSpeedSetpoint + delta));
    }

    private void adjustRearSpeed(double delta) {
        rearSpeedSetpoint = Math.min(1.0, Math.max(0.0, rearSpeedSetpoint + delta));
    }

    private void runReverse() {
        frontMotor.set(context.getShooterFrontReverseSpeed());
        rearMotor.set(context.getShooterRearReverseSpeed());
    }

    private void stop() {
        frontMotor.set(0);
        rearMotor.set(0);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Shoot command — spins both flywheels to fire balls toward the hub.
     *
     * <p>Intended to run in parallel with {@code RelaySubsystem.getRunCommand()} and
     * {@code IndexerSubsystem.getIndexCommand()} via a {@code Commands.parallel()} group
     * on the manipulator right trigger.
     * Held-button: runs while held, stops on release.
     *
     * <p><b>Note:</b> Current implementation uses open-loop percent output. Once the
     * physics-based inverse solver from SHOOTER.md is integrated, this command will accept
     * a distance-to-target supplier and set flywheel RPM targets via PID closed-loop.
     *
     * @return Command that runs the shooter while held
     */
    public Command getShootCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.SHOOTING);
                            runForward();
                        },
                        () -> {
                            stop();
                            if (currentState == State.SHOOTING) setState(State.IDLE);
                        })
                .withName("Shooter.Shoot");
    }

    /**
     * Reverse command — spins both flywheels in reverse to clear jams or eject balls.
     *
     * <p>Intended to run in parallel with {@code RelaySubsystem.getReverseCommand()} and
     * {@code IndexerSubsystem.getReverseCommand()} via a {@code Commands.parallel()} group
     * on the manipulator right bumper.
     * Held-button: runs while held, stops on release.
     *
     * @return Command that reverses the shooter while held
     */
    public Command getReverseCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.REVERSING);
                            runReverse();
                        },
                        () -> {
                            stop();
                            if (currentState == State.REVERSING) setState(State.IDLE);
                        })
                .withName("Shooter.Reverse");
    }

    /**
     * Stop command — stops both flywheel motors (emergency / safe shutdown).
     *
     * @return Command that stops the shooter
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {
                    stop();
                    setState(State.IDLE);
                })
                .withName("Shooter.Stop");
    }

    /**
     * Increases the front flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 1.0).
     *
     * <p>Does <em>not</em> require the shooter subsystem, so it can be tapped while
     * {@link #getShootCommand()} is held — the running command reads the updated setpoint
     * on its next loop.
     *
     * @return Instant command that bumps the front setpoint up one step
     */
    public Command getIncreaseFrontSpeedCommand() {
        return Commands.runOnce(() -> adjustFrontSpeed(SPEED_STEP)).withName("Shooter.FrontSpeed+");
    }

    /**
     * Decreases the front flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 0.0).
     *
     * @return Instant command that bumps the front setpoint down one step
     */
    public Command getDecreaseFrontSpeedCommand() {
        return Commands.runOnce(() -> adjustFrontSpeed(-SPEED_STEP)).withName("Shooter.FrontSpeed-");
    }

    /**
     * Increases the rear flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 1.0).
     *
     * @return Instant command that bumps the rear setpoint up one step
     */
    public Command getIncreaseRearSpeedCommand() {
        return Commands.runOnce(() -> adjustRearSpeed(SPEED_STEP)).withName("Shooter.RearSpeed+");
    }

    /**
     * Decreases the rear flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 0.0).
     *
     * @return Instant command that bumps the rear setpoint down one step
     */
    public Command getDecreaseRearSpeedCommand() {
        return Commands.runOnce(() -> adjustRearSpeed(-SPEED_STEP)).withName("Shooter.RearSpeed-");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(
                TELEMETRY_PREFIX + "/Started",
                "FrontMotorID=" + context.getShooterFrontMotorId() + " RearMotorID=" + context.getShooterRearMotorId());
    }

    private void captureTelemetry(String prefix) {
        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Front/OutputPercent", frontMotor.getAppliedOutput(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Rear/OutputPercent", rearMotor.getAppliedOutput(), TelemetryLevel.MATCH);

        // LAB level
        Telemetry.record(prefix + "/Front/Current", frontMotor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/Rear/Current", rearMotor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/Front/SpeedSetpoint", frontSpeedSetpoint, TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/Rear/SpeedSetpoint", rearSpeedSetpoint, TelemetryLevel.LAB);

        // VERBOSE level
        if (RobotBase.isSimulation()) {
            Telemetry.record(
                    prefix + "/Front/VelocityRPM", frontMotorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
            Telemetry.record(
                    prefix + "/Rear/VelocityRPM", rearMotorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
        }
        // Real robot velocity telemetry (LAB level)
        try {
            double frontRpm = frontMotor.getEncoder().getVelocity();
            double rearRpm = rearMotor.getEncoder().getVelocity();
            Telemetry.publish(prefix + "/Front/VelocityRPM", frontRpm, TelemetryLevel.LAB);
            Telemetry.publish(prefix + "/Rear/VelocityRPM", rearRpm, TelemetryLevel.LAB);
        } catch (Exception ignore) {
            // Encoder may not be available in some build configs — ignore telemetry in that case.
        }
        Telemetry.publish(prefix + "/Front/TargetRPM", targetFrontRPM, TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/Rear/TargetRPM", targetRearRPM, TelemetryLevel.LAB);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Refresh gains from Shuffleboard / NetworkTables so they can be tuned live
        double kp = kPEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kP);
        double ki = kIEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kI);
        double kd = kDEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_kD);
        double ks = kSEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KS);
        double kv = kVEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KV);
        double ka = kAEntry.getDouble(frc.robot.Constants.ShooterConstants.SHOOTER_KA);

        // Apply gains to controllers and feedforward instances
        frontPid.setP(kp);
        frontPid.setI(ki);
        frontPid.setD(kd);
        rearPid.setP(kp);
        rearPid.setI(ki);
        rearPid.setD(kd);

        this.frontFeedforward = new SimpleMotorFeedforward(ks, kv, ka);
        this.rearFeedforward = new SimpleMotorFeedforward(ks, kv, ka);

        // If closed-loop control is enabled, compute feedforward+PID and apply voltages.
        if (closedLoopEnabled) {
            // Measure RPMs
            double measuredFrontRPM = RobotBase.isSimulation() && frontMotorSim != null
                    ? frontMotorSim.getAngularVelocityRPM()
                    : frontMotor.getEncoder().getVelocity();
            double measuredRearRPM = RobotBase.isSimulation() && rearMotorSim != null
                    ? rearMotorSim.getAngularVelocityRPM()
                    : rearMotor.getEncoder().getVelocity();

            // Feedforward expects angular velocity (rad/s). Convert RPM -> rad/s.
            double targetFrontRadPerSec = targetFrontRPM / 60.0 * 2.0 * Math.PI;
            double targetRearRadPerSec = targetRearRPM / 60.0 * 2.0 * Math.PI;

            double ffFront = frontFeedforward.calculate(targetFrontRadPerSec);
            double ffRear = rearFeedforward.calculate(targetRearRadPerSec);

            // PID controllers operate in RPM space (gains are volts per RPM)
            double pidFront = frontPid.calculate(measuredFrontRPM, targetFrontRPM);
            double pidRear = rearPid.calculate(measuredRearRPM, targetRearRPM);

            double outFrontVolts = ffFront + pidFront;
            double outRearVolts = ffRear + pidRear;

            // Clamp to battery voltage for safety
            double vmax = RobotController.getBatteryVoltage();
            outFrontVolts = Math.max(-vmax, Math.min(vmax, outFrontVolts));
            outRearVolts = Math.max(-vmax, Math.min(vmax, outRearVolts));

            // Apply voltages
            try {
                frontMotor.setVoltage(outFrontVolts);
                rearMotor.setVoltage(outRearVolts);
            } catch (Exception e) {
                // Fallback to percent output if voltage API unavailable
                frontMotor.set(outFrontVolts / Math.max(0.1, vmax));
                rearMotor.set(outRearVolts / Math.max(0.1, vmax));
            }
        }
    }

    /**
     * Returns a command that uses Limelight AprilTag pose to compute a target RPM and
     * runs closed-loop shooter control while held.
     * Uses constants in {@link frc.robot.Constants.ShooterConstants} for mapping.
     */
    public Command getShootWithLimelightCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.SHOOTING);
                            closedLoopEnabled = true;
                            // Sample Limelight pose and compute target RPM
                            try {
                                if (frc.robot.support.limelight.LimelightHelpers.getTargetCount("limelight-front")
                                        > 0) {
                                    var pose3d =
                                            frc.robot.support.limelight.LimelightHelpers.getTargetPose3d_RobotSpace(
                                                    "limelight-front");
                                    double distance = pose3d.getTranslation().getNorm();
                                    double rpm = frc.robot.Constants.ShooterConstants.SHOOTER_RPM_OFFSET
                                            + frc.robot.Constants.ShooterConstants.SHOOTER_RPM_PER_METER * distance;
                                    setTargetRPMs(rpm, rpm);
                                }
                            } catch (Exception ignore) {
                                // Don't update target if limelight data unavailable
                            }
                        },
                        () -> {
                            closedLoopEnabled = false;
                            stop();
                            if (currentState == State.SHOOTING) setState(State.IDLE);
                        })
                .withName("Shooter.VisionShoot");
    }

    // -------------------------------------------------------------------------
    // Closed-loop helpers & command factories
    // -------------------------------------------------------------------------

    /** Set both target RPMs for closed-loop control. */
    public void setTargetRPMs(double frontRpm, double rearRpm) {
        this.targetFrontRPM = frontRpm;
        this.targetRearRPM = rearRpm;
    }

    /** Convenience: set both targets to the same RPM. */
    public void setTargetRPM(double rpm) {
        setTargetRPMs(rpm, rpm);
    }

    /** Returns the current measured front flywheel velocity in RPM (best-effort). */
    public double getFrontVelocityRPM() {
        if (RobotBase.isSimulation() && frontMotorSim != null) return frontMotorSim.getAngularVelocityRPM();
        try {
            return frontMotor.getEncoder().getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    /** Returns the current measured rear flywheel velocity in RPM (best-effort). */
    public double getRearVelocityRPM() {
        if (RobotBase.isSimulation() && rearMotorSim != null) return rearMotorSim.getAngularVelocityRPM();
        try {
            return rearMotor.getEncoder().getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    /**
     * Returns a command that runs closed-loop velocity control while held. The provided suppliers
     * are sampled each scheduler iteration and the subsystem periodic loop applies voltages.
     */
    public Command getShootRPMCommand(DoubleSupplier frontRpmSupplier, DoubleSupplier rearRpmSupplier) {
        return this.runEnd(
                        () -> {
                            setState(State.SHOOTING);
                            closedLoopEnabled = true;
                            setTargetRPMs(frontRpmSupplier.getAsDouble(), rearRpmSupplier.getAsDouble());
                        },
                        () -> {
                            closedLoopEnabled = false;
                            stop();
                            if (currentState == State.SHOOTING) setState(State.IDLE);
                        })
                .withName("Shooter.ShootRPM");
    }

    /** Convenience: fixed-target RPM command (both wheels same RPM). */
    public Command getShootRPMCommand(double rpm) {
        return getShootRPMCommand(() -> rpm, () -> rpm);
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    /**
     * Advances simulated flywheel physics each tick.
     *
     * <p>The shooter has no hardstops — this propagates the applied voltage through each
     * NEO DCMotorSim independently to produce realistic velocity and current readings for
     * telemetry. The difference in simulated RPM between front (3") and rear (4") flywheels
     * reflects the diameter-driven surface speed asymmetry described in SHOOTER.md.
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        // Front flywheel (A, 3")
        double frontVoltage = frontSparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        frontMotorSim.setInputVoltage(frontVoltage);
        frontMotorSim.update(dt);
        frontSparkMaxSim.iterate(frontMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);

        // Rear flywheel (B, 4")
        double rearVoltage = rearSparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        rearMotorSim.setInputVoltage(rearVoltage);
        rearMotorSim.update(dt);
        rearSparkMaxSim.iterate(rearMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
    }
}
