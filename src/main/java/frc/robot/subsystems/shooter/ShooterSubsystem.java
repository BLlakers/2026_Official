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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.function.DoubleSupplier;

/**
 * Shooter subsystem — single-flywheel launcher.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → ball fired toward the target (hub)</li>
 *   <li>Negative output → reverse to clear jams or eject balls back toward indexer</li>
 * </ul>
 *
 * <h2>Coordination</h2>
 * <p>The shooter is commanded in unison with the relay and indexer via a
 * {@code Commands.parallel()} group bound to the manipulator right trigger.
 */
public class ShooterSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Shooter";

    /**
     * Step size applied to the flywheel speed setpoint on each increase/decrease command tap.
     * 0.02 = 2% motor output per tap → 50 steps across the full [0, 1] range.
     */
    private static final double SPEED_STEP = 0.02;

    /** Operating states of the shooter mechanism. */
    public enum State {
        /** Flywheel motor stopped. */
        IDLE,
        /** Flywheel motor spinning to fire balls. */
        SHOOTING,
        /** Flywheel motor spinning in reverse to clear jams / eject balls. */
        REVERSING,
    }

    private final ShooterSubsystemContext context;

    private final SparkMax motor;

    // Runtime-adjustable speed setpoint — initialised from context, tunable via debug commands.
    private double speedSetpoint;

    // Closed-loop control fields (feedforward + PID)
    private SimpleMotorFeedforward feedforward;
    private final PIDController pid;

    // SmartDashboard keys for live tuning via Shuffleboard
    private static final String SD_KP = "Shooter/kP";
    private static final String SD_KI = "Shooter/kI";
    private static final String SD_KD = "Shooter/kD";
    private static final String SD_KS = "Shooter/kS";
    private static final String SD_KV = "Shooter/kV";
    private static final String SD_KA = "Shooter/kA";
    private static final String SD_TARGET_RPM = "Shooter/TargetRPM";

    // Target RPM for closed-loop control
    private double targetRPM = 0.0;

    // Whether closed-loop velocity control is active
    private boolean closedLoopEnabled = false;

    private State currentState = State.IDLE;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim motorSim;
    private SparkMaxSim sparkMaxSim;
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

        this.motor = new SparkMax(context.getShooterMotorId(), MotorType.kBrushless);

        this.speedSetpoint = context.getShooterSpeed();

        configureMotor();

        // Initialize PID controller (gains will be updated from Shuffleboard entries at runtime)
        this.pid = new PIDController(
                frc.robot.Constants.ShooterConstants.SHOOTER_kP,
                frc.robot.Constants.ShooterConstants.SHOOTER_kI,
                frc.robot.Constants.ShooterConstants.SHOOTER_kD);

        // Seed SmartDashboard with defaults — editable from Shuffleboard at runtime
        SmartDashboard.putNumber(SD_KP, frc.robot.Constants.ShooterConstants.SHOOTER_kP);
        SmartDashboard.putNumber(SD_KI, frc.robot.Constants.ShooterConstants.SHOOTER_kI);
        SmartDashboard.putNumber(SD_KD, frc.robot.Constants.ShooterConstants.SHOOTER_kD);
        SmartDashboard.putNumber(SD_KS, frc.robot.Constants.ShooterConstants.SHOOTER_KS);
        SmartDashboard.putNumber(SD_KV, frc.robot.Constants.ShooterConstants.SHOOTER_KV);
        SmartDashboard.putNumber(SD_KA, frc.robot.Constants.ShooterConstants.SHOOTER_KA);
        SmartDashboard.putNumber(SD_TARGET_RPM, 3000.0);

        // Initialize feedforward from constants (refreshed from NT in periodic())
        this.feedforward = new SimpleMotorFeedforward(
                frc.robot.Constants.ShooterConstants.SHOOTER_KS,
                frc.robot.Constants.ShooterConstants.SHOOTER_KV,
                frc.robot.Constants.ShooterConstants.SHOOTER_KA);

        if (RobotBase.isSimulation()) {
            this.sparkMaxSim = new SparkMaxSim(motor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> plant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.001, context.getShooterGearRatio());
            this.motorSim = new DCMotorSim(plant, DCMotor.getNEO(1));

            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(context.getShooterCurrentLimit());
        config.idleMode(IdleMode.kCoast);
        config.inverted(context.isShooterMotorInverted());
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
        motor.set(Constants.ShooterConstants.SHOOTER_SPEED);
    }

    private void adjustSpeed(double delta) {
        speedSetpoint = Math.min(1.0, Math.max(0.0, speedSetpoint + delta));
    }

    private void runReverse() {
        motor.set(context.getShooterReverseSpeed());
    }

    private void stop() {
        motor.set(0);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Shoot command — spins the flywheel to fire balls toward the hub.
     *
     * <p>Held-button: runs while held, stops on release.
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
     * Reverse command — spins the flywheel in reverse to clear jams or eject balls.
     *
     * <p>Held-button: runs while held, stops on release.
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
     * Stop command — stops the flywheel motor (emergency / safe shutdown).
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
     * Increases the flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 1.0).
     *
     * @return Instant command that bumps the setpoint up one step
     */
    public Command getIncreaseSpeedCommand() {
        return Commands.runOnce(() -> adjustSpeed(SPEED_STEP)).withName("Shooter.Speed+");
    }

    /**
     * Decreases the flywheel speed setpoint by {@value #SPEED_STEP} (clamped to 0.0).
     *
     * @return Instant command that bumps the setpoint down one step
     */
    public Command getDecreaseSpeedCommand() {
        return Commands.runOnce(() -> adjustSpeed(-SPEED_STEP)).withName("Shooter.Speed-");
    }

    private double getDistanceToTag() {
        var pose3d = frc.robot.support.limelight.LimelightHelpers.getTargetPose3d_RobotSpace("limelight-front");
        return pose3d.getTranslation().getNorm();
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "MotorID=" + context.getShooterMotorId());
    }

    private void captureTelemetry(String prefix) {
        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/OutputPercent", motor.getAppliedOutput(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "DistanceToTag", getDistanceToTag(), TelemetryLevel.NONE);
        // LAB level
        Telemetry.record(prefix + "/Current", motor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/SpeedSetpoint", speedSetpoint, TelemetryLevel.LAB);

        // VERBOSE level
        if (RobotBase.isSimulation()) {
            Telemetry.record(prefix + "/VelocityRPM", motorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
        }
        // Real robot velocity telemetry (LAB level)
        try {
            double rpm = motor.getEncoder().getVelocity();
            Telemetry.publish(prefix + "/VelocityRPM", rpm, TelemetryLevel.LAB);
        } catch (Exception ignore) {
            // Encoder may not be available in some build configs
        }
        Telemetry.publish(prefix + "/TargetRPM", targetRPM, TelemetryLevel.LAB);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Refresh gains from SmartDashboard so they can be tuned live in Shuffleboard
        double kp = SmartDashboard.getNumber(SD_KP, frc.robot.Constants.ShooterConstants.SHOOTER_kP);
        double ki = SmartDashboard.getNumber(SD_KI, frc.robot.Constants.ShooterConstants.SHOOTER_kI);
        double kd = SmartDashboard.getNumber(SD_KD, frc.robot.Constants.ShooterConstants.SHOOTER_kD);
        double ks = SmartDashboard.getNumber(SD_KS, frc.robot.Constants.ShooterConstants.SHOOTER_KS);
        double kv = SmartDashboard.getNumber(SD_KV, frc.robot.Constants.ShooterConstants.SHOOTER_KV);
        double ka = SmartDashboard.getNumber(SD_KA, frc.robot.Constants.ShooterConstants.SHOOTER_KA);

        // Apply gains to controller and feedforward
        pid.setP(kp);
        pid.setI(ki);
        pid.setD(kd);

        this.feedforward = new SimpleMotorFeedforward(ks, kv, ka);

        // If closed-loop control is enabled, compute feedforward+PID and apply voltage.
        if (closedLoopEnabled) {
            double measuredRPM = RobotBase.isSimulation() && motorSim != null
                    ? motorSim.getAngularVelocityRPM()
                    : motor.getEncoder().getVelocity();

            // Feedforward expects angular velocity (rad/s). Convert RPM -> rad/s.
            double targetRadPerSec = targetRPM / 60.0 * 2.0 * Math.PI;

            double ff = feedforward.calculate(targetRadPerSec);
            double pidOutput = pid.calculate(measuredRPM, targetRPM);

            double outVolts = ff + pidOutput;

            // Clamp to battery voltage for safety
            double vmax = RobotController.getBatteryVoltage();
            outVolts = Math.max(-vmax, Math.min(vmax, outVolts));

            // Apply voltage
            try {
                motor.setVoltage(outVolts);
            } catch (Exception e) {
                motor.set(outVolts / Math.max(0.1, vmax));
            }
        }
    }

    /**
     * Returns a command that uses Limelight AprilTag pose to compute a target RPM and
     * runs closed-loop shooter control while held.
     */
    public Command getShootWithLimelightCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.SHOOTING);
                            closedLoopEnabled = true;
                            try {
                                if (frc.robot.support.limelight.LimelightHelpers.getTargetCount("limelight-front")
                                        > 0) {
                                    //                                    var pose3d =
                                    //
                                    // frc.robot.support.limelight.LimelightHelpers.getTargetPose3d_RobotSpace(
                                    //                                                    "limelight-front");
                                    //                                    double distance =
                                    // pose3d.getTranslation().getNorm();
                                    double distance = getDistanceToTag();
                                    double rpm = frc.robot.Constants.ShooterConstants.SHOOTER_RPM_OFFSET
                                            + frc.robot.Constants.ShooterConstants.SHOOTER_RPM_PER_METER * distance;
                                    setTargetRPM(rpm);
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

    /** Set target RPM for closed-loop control. */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /** Returns the current measured flywheel velocity in RPM (best-effort). */
    public double getVelocityRPM() {
        if (RobotBase.isSimulation() && motorSim != null) return motorSim.getAngularVelocityRPM();
        try {
            return motor.getEncoder().getVelocity();
        } catch (Exception e) {
            return 0.0;
        }
    }

    /**
     * Returns a command that runs closed-loop velocity control while held.
     */
    public Command getShootRPMCommand(DoubleSupplier rpmSupplier) {
        return this.runEnd(
                        () -> {
                            setState(State.SHOOTING);
                            closedLoopEnabled = true;
                            setTargetRPM(rpmSupplier.getAsDouble());
                        },
                        () -> {
                            closedLoopEnabled = false;
                            stop();
                            if (currentState == State.SHOOTING) setState(State.IDLE);
                        })
                .withName("Shooter.ShootRPM");
    }

    /** Convenience: fixed-target RPM command. */
    public Command getShootRPMCommand(double rpm) {
        return getShootRPMCommand(() -> rpm);
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------

    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        double voltage = sparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        motorSim.setInputVoltage(voltage);
        motorSim.update(dt);
        sparkMaxSim.iterate(motorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
    }
}
