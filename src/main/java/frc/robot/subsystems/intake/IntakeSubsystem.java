package frc.robot.subsystems.intake;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;
import static java.util.Objects.requireNonNull;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkFlexSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * Intake subsystem for collecting fuel balls off the ground.
 *
 * <h2>Mechanism Overview</h2>
 * <p>A single NEO Vortex (1:1) drives an intake roller chain that lifts balls and delivers them
 * into a fabric bag void on the robot. One roller is directly driven by the motor shaft; the
 * second is connected via chain drive from the first.
 *
 * <p>A 2-motor lift (25:1 NEO × 2, one per side) articulates the entire hopper assembly up and
 * down. The hopper is lowered for most of the match and raised (stowed) before and during climb
 * to satisfy the frame-perimeter size rule.
 *
 * <h2>Encoder Convention (Lift)</h2>
 * <ul>
 *   <li>Encoder = 0 → fully retracted (hopper at retracted hardstop — homing reference)</li>
 *   <li>Encoder negative → hopper lowered toward match position</li>
 * </ul>
 *
 * <h2>Homing</h2>
 * <p>Both lift motors slowly raise the hopper until <em>either</em> motor detects a current spike
 * (retracted hardstop contact). Both encoders are then zeroed. Re-homing mid-match is expected —
 * the encoders are relative and may drift after collisions.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm lift motor inversion — see {@link IntakeSubsystemContext#isLiftMotor1Inverted()}
 *       and {@link IntakeSubsystemContext#isLiftMotor2Inverted()}</li>
 *   <li>Confirm roller motor direction — positive output should intake balls inward</li>
 *   <li>Tune {@code homingCurrentThresholdAmps} by watching
 *       {@code Intake/Lift/Motor1/Current} and {@code Intake/Lift/Motor2/Current}</li>
 *   <li>Measure {@code raisedPositionRotations} on physical robot</li>
 *   <li>Tune all speed constants</li>
 * </ul>
 */
public class IntakeSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Intake";

    /** Operating states of the hopper mechanism. */
    public enum State {
        /** All motors stopped; encoder position unknown. Safe before homing. */
        IDLE,
        /** Lift slowly raising to find retracted hardstop and zero encoders. */
        HOMING,
        /** Intake at lowered (match) position; rollers stopped. */
        LOWERED,
        /** Intake at lowered position; rollers spinning inward to collect balls. */
        FEEDING,
        /** Intake at lowered position; rollers spinning outward to eject. */
        REVERSING,
        /** Lift moving upward toward stowed position. */
        RAISING,
        /** Intake at raised (stowed) position; all motors holding. */
        RAISED,
    }

    private final IntakeSubsystemContext context;

    // Motors
    private final SparkFlex rollerMotor; // NEO Vortex
    private final SparkMax liftMotor1; // NEO — right side
    private final SparkMax liftMotor2; // NEO — left side

    // Encoders (lift only — roller has no position control)
    private final RelativeEncoder lift1Encoder;
    private final RelativeEncoder lift2Encoder;

    private final ProfiledPIDController liftController;

    private State currentState = State.IDLE;

    /** The lift encoder target currently being sought. Used for telemetry. */
    private double targetRotations = 0.0;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim lift1MotorSim;
    private DCMotorSim lift2MotorSim;
    private SparkMaxSim lift1SparkMaxSim;
    private SparkMaxSim lift2SparkMaxSim;
    private SparkFlexSim rollerSparkFlexSim;

    private double simLift1Position = 0.0;
    private double simLift2Position = 0.0;
    private double simLift1Current = 0.0;
    private double simLift2Current = 0.0;
    private double lastSimTime = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates a IntakeSubsystem with default context.
     */
    public IntakeSubsystem() {
        this(IntakeSubsystemContext.defaults());
    }

    /**
     * Instantiates a IntakeSubsystem with the specified context.
     *
     * @param context The IntakeSubsystemContext to apply to this instance
     */
    public IntakeSubsystem(final IntakeSubsystemContext context) {
        requireNonNull(context, "IntakeSubsystemContext cannot be null");
        this.context = context;

        this.rollerMotor = new SparkFlex(context.getRollerMotorId(), MotorType.kBrushless);
        this.liftMotor1 = new SparkMax(context.getLiftMotor1Id(), MotorType.kBrushless);
        this.liftMotor2 = new SparkMax(context.getLiftMotor2Id(), MotorType.kBrushless);

        this.lift1Encoder = liftMotor1.getEncoder();
        this.lift2Encoder = liftMotor2.getEncoder();

        this.liftController = new ProfiledPIDController(
                context.getLiftPid().p(),
                context.getLiftPid().i(),
                context.getLiftPid().d(),
                new TrapezoidProfile.Constraints(
                        context.getLiftMaxVelocityRotsPerSec(), context.getLiftMaxAccelerationRotsPerSecSq()));
        liftController.setTolerance(context.getPositionToleranceRotations());

        configureRollerMotor();
        configureLiftMotors();

        if (RobotBase.isSimulation()) {
            this.lift1SparkMaxSim = new SparkMaxSim(liftMotor1, DCMotor.getNEO(1));
            this.lift2SparkMaxSim = new SparkMaxSim(liftMotor2, DCMotor.getNEO(1));
            this.rollerSparkFlexSim = new SparkFlexSim(rollerMotor, DCMotor.getNeoVortex(1));
            LinearSystem<N2, N1, N2> liftPlant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.01, context.getLiftGearRatio());
            this.lift1MotorSim = new DCMotorSim(liftPlant, DCMotor.getNEO(1));
            this.lift2MotorSim = new DCMotorSim(liftPlant, DCMotor.getNEO(1));
            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureRollerMotor() {
        SparkFlexConfig config = new SparkFlexConfig();
        config.smartCurrentLimit(context.getRollerCurrentLimit());
        config.idleMode(IdleMode.kCoast); // Coast so rollers don't snap-stop and jam balls
        // TODO: Set config.inverted(true/false) once roller direction is confirmed with build team.
        //       Convention: positive output = rollers spin inward to collect balls.
        config.inverted(context.isRollorMotorInverted());
        rollerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    private void configureLiftMotors() {
        SparkMaxConfig config1 = new SparkMaxConfig();
        config1.smartCurrentLimit(context.getLiftCurrentLimit());
        config1.idleMode(IdleMode.kBrake); // Hold position when stopped
        config1.inverted(context.isLiftMotor1Inverted());
        liftMotor1.configure(config1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig config2 = new SparkMaxConfig();
        config2.smartCurrentLimit(context.getLiftCurrentLimit());
        config2.idleMode(IdleMode.kBrake);
        config2.inverted(context.isLiftMotor2Inverted());
        liftMotor2.configure(config2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // Sim-aware sensor helpers
    // -------------------------------------------------------------------------

    /**
     * Returns the average lift position across both encoders.
     * In simulation, routes through tracked sim state instead of real encoder.
     */
    private double getLiftPosition() {
        if (RobotBase.isSimulation()) {
            return (simLift1Position + simLift2Position) / 2.0;
        }
        return (lift1Encoder.getPosition() + lift2Encoder.getPosition()) / 2.0;
    }

    private double getLift1Current() {
        return RobotBase.isSimulation() ? simLift1Current : liftMotor1.getOutputCurrent();
    }

    private double getLift2Current() {
        return RobotBase.isSimulation() ? simLift2Current : liftMotor2.getOutputCurrent();
    }

    // -------------------------------------------------------------------------
    // State & position helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    /**
     * Returns the current operating state of the hopper.
     *
     * @return Current {@link State}
     */
    public State getState() {
        return currentState;
    }

    /**
     * Returns whether homing has been completed (encoders are zeroed and mechanism is safe to use).
     *
     * @return true if state is not IDLE or HOMING
     */
    public boolean isHomed() {
        return currentState != State.IDLE && currentState != State.HOMING;
    }

    /** Returns true when the lift is within tolerance of the given target. */
    private boolean atTarget(double target) {
        return Math.abs(getLiftPosition() - target) <= context.getPositionToleranceRotations();
    }

    // -------------------------------------------------------------------------
    // Motor actions (private — exposed through command factories)
    // -------------------------------------------------------------------------

    private void spinRollersIn() {
        rollerMotor.set(context.getIntakeSpeed());
    }

    private void spinRollersOut() {
        rollerMotor.set(context.getReverseSpeed());
    }

    private void stopRollers() {
        rollerMotor.set(0);
    }

    private void raiseLift() {
        liftMotor1.set(context.getRaiseSpeed());
        liftMotor2.set(context.getRaiseSpeed());
    }

    private void lowerLift() {
        liftMotor1.set(context.getLowerSpeed());
        liftMotor2.set(context.getLowerSpeed());
    }

    private void homingRaiseLift() {
        liftMotor1.set(context.getHomingSpeed());
        liftMotor2.set(context.getHomingSpeed());
    }

    private void holdLift() {
        liftMotor1.set(0);
        liftMotor2.set(0);
    }

    /** Applies the same duty-cycle output to both lift motors. */
    private void applyLiftOutput(double output) {
        liftMotor1.set(output);
        liftMotor2.set(output);
    }

    private void stopAll() {
        stopRollers();
        holdLift();
    }

    // -------------------------------------------------------------------------
    // Homing helpers
    // -------------------------------------------------------------------------

    /**
     * Returns true when a current spike on either lift motor indicates the hopper has
     * contacted the retracted hardstop during homing.
     */
    private boolean isAtRetractedHardstop() {
        return getLift1Current() >= context.getHomingCurrentThresholdAmps()
                || getLift2Current() >= context.getHomingCurrentThresholdAmps();
    }

    /**
     * Returns true when a current spike on either lift motor indicates the intake has
     * contacted the extended (lower) hardstop.
     */
    private boolean isAtExtendedHardstop() {
        return getLift1Current() >= context.getExtendedHardstopCurrentThresholdAmps()
                || getLift2Current() >= context.getExtendedHardstopCurrentThresholdAmps();
    }

    /** Zeros both lift encoders and marks homing complete after retracted hardstop contact. */
    private void completeHoming() {
        holdLift();
        lift1Encoder.setPosition(0.0);
        lift2Encoder.setPosition(0.0);
        if (RobotBase.isSimulation()) {
            simLift1Position = 0.0;
            simLift2Position = 0.0;
            lift1MotorSim.setState(VecBuilder.fill(0.0, 0.0));
            lift2MotorSim.setState(VecBuilder.fill(0.0, 0.0));
        }
        targetRotations = 0.0;
        setState(State.RAISED);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Homing command — slowly raises the hopper until it contacts the retracted hardstop (current
     * spike on either lift motor), then zeroes both encoders.
     *
     * <p>After homing: encoder = 0 = fully retracted (hardstop contact). Negative = lowered.
     *
     * <p>This command is safe to run mid-match whenever the lift encoder is suspected to have
     * drifted (e.g., after a collision). The hopper must not be carrying the robot when homing.
     *
     * @return Command that homes the hopper lift
     */
    public Command getHomingCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.HOMING);
                            homingRaiseLift();
                        },
                        this::holdLift)
                .until(this::isAtRetractedHardstop)
                .andThen(this.runOnce(this::completeHoming))
                .withName("Intake.Home");
    }

    /**
     * Raise command — lifts the hopper to the stowed (raised) position using a
     * ProfiledPIDController for smooth trapezoidal motion.
     *
     * <p>Used before and during climb to retract the hopper within the frame perimeter.
     * Stops rollers on entry. The controller is reset from the current position to avoid
     * velocity jumps if the command is interrupted and re-triggered mid-travel.
     *
     * @return Command that raises the hopper to stowed position
     */
    public Command getRaiseCommand() {
        return this.runOnce(() -> {
                    stopRollers();
                    liftController.reset(getLiftPosition());
                    liftController.setGoal(context.getRaisedPositionRotations());
                    setState(State.RAISING);
                })
                .andThen(this.run(() -> {
                    targetRotations = context.getRaisedPositionRotations();
                    applyLiftOutput(liftController.calculate(getLiftPosition()));
                }))
                .until(liftController::atGoal)
                .andThen(this.runOnce(() -> {
                    holdLift();
                    setState(State.RAISED);
                }))
                .withName("Intake.Raise");
    }

    /**
     * Lower command — returns the hopper to the match (lowered) position using a
     * ProfiledPIDController for smooth trapezoidal motion.
     *
     * <p>Terminates when either the position goal is reached ({@code atGoal()}) OR a current
     * spike signals physical contact with the extended (lower) hardstop. This prevents motor
     * stall if the mechanism reaches the floor before the encoder reads at the exact setpoint.
     *
     * @return Command that lowers the hopper to match position
     */
    public Command getLowerCommand() {
        return this.runOnce(() -> {
                    liftController.reset(getLiftPosition());
                    liftController.setGoal(context.getLoweredPositionRotations());
                })
                .andThen(this.run(() -> {
                    targetRotations = context.getLoweredPositionRotations();
                    applyLiftOutput(liftController.calculate(getLiftPosition()));
                }))
                .until(() -> liftController.atGoal() || isAtExtendedHardstop())
                .andThen(this.runOnce(() -> {
                    holdLift();
                    setState(State.LOWERED);
                }))
                .withName("Intake.Lower");
    }

    /**
     * Intake command — runs the rollers inward to collect fuel balls.
     *
     * <p>Held-button command: rollers spin while the button is held, stop on release.
     * Intended for use while the hopper is lowered; no lift movement is commanded.
     *
     * @return Command that runs the intake rollers while held
     */
    public Command getIntakeCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.FEEDING);
                            spinRollersIn();
                        },
                        () -> {
                            stopRollers();
                            if (currentState == State.FEEDING) setState(State.LOWERED);
                        })
                .withName("Intake.Intake");
    }

    /**
     * Reverse command — runs the rollers outward to eject balls.
     *
     * <p>Held-button command: rollers spin in reverse while held, stop on release.
     *
     * @return Command that reverses the intake rollers while held
     */
    public Command getReverseCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.REVERSING);
                            spinRollersOut();
                        },
                        () -> {
                            stopRollers();
                            if (currentState == State.REVERSING) setState(State.LOWERED);
                        })
                .withName("Intake.Reverse");
    }

    /**
     * Manual raise command — held-button override for raising the hopper.
     * Holds position on release.
     *
     * @return Command that manually raises the hopper while held
     */
    public Command getManualRaiseCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.RAISING);
                            raiseLift();
                        },
                        () -> {
                            holdLift();
                            setState(State.RAISED);
                        })
                .withName("Intake.ManualRaise");
    }

    /**
     * Manual lower command — held-button override for lowering the hopper.
     * Holds position on release.
     *
     * @return Command that manually lowers the hopper while held
     */
    public Command getManualLowerCommand() {
        return this.runEnd(() -> lowerLift(), () -> {
                    holdLift();
                    setState(State.LOWERED);
                })
                .withName("Intake.ManualLower");
    }

    /**
     * Stop command — stops all motors (emergency / safe shutdown).
     *
     * @return Command that stops the entire hopper mechanism
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {
                    stopAll();
                    setState(State.IDLE);
                })
                .withName("Intake.Stop");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(
                TELEMETRY_PREFIX + "/Started",
                "RollerID=" + context.getRollerMotorId()
                        + " Lift1ID=" + context.getLiftMotor1Id()
                        + " Lift2ID=" + context.getLiftMotor2Id());
    }

    private void captureTelemetry(String prefix) {
        double liftPos = getLiftPosition();

        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Lift/Position", liftPos, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Lift/Target", targetRotations, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Lift/AtTarget", atTarget(targetRotations) ? 1.0 : 0.0, TelemetryLevel.MATCH);

        // LAB level
        Telemetry.record(prefix + "/Lift/Motor1/Current", getLift1Current(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/Lift/Motor2/Current", getLift2Current(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/Roller/OutputPercent", rollerMotor.getAppliedOutput(), TelemetryLevel.LAB);
        Telemetry.record(
                prefix + "/Homing/CurrentThreshold", context.getHomingCurrentThresholdAmps(), TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/Lift/PID/SetpointPosition", liftController.getSetpoint().position, TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/Lift/PID/SetpointVelocity", liftController.getSetpoint().velocity, TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/Lift/PID/AtGoal", liftController.atGoal() ? 1.0 : 0.0, TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/Lift/ExtendedHardstop/Active", isAtExtendedHardstop() ? 1.0 : 0.0, TelemetryLevel.LAB);
        Telemetry.publish(
                prefix + "/Homing/ExtendedCurrentThreshold",
                context.getExtendedHardstopCurrentThresholdAmps(),
                TelemetryLevel.LAB);

        // VERBOSE level
        Telemetry.record(prefix + "/Lift/Motor1/OutputPercent", liftMotor1.getAppliedOutput(), TelemetryLevel.VERBOSE);
        Telemetry.record(prefix + "/Lift/Motor2/OutputPercent", liftMotor2.getAppliedOutput(), TelemetryLevel.VERBOSE);
        Telemetry.record(
                prefix + "/Lift/Motor1/EncoderRaw",
                RobotBase.isSimulation() ? simLift1Position : lift1Encoder.getPosition(),
                TelemetryLevel.VERBOSE);
        Telemetry.record(
                prefix + "/Lift/Motor2/EncoderRaw",
                RobotBase.isSimulation() ? simLift2Position : lift2Encoder.getPosition(),
                TelemetryLevel.VERBOSE);
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
     * Advances simulated lift motor physics each tick.
     *
     * <p>Mirrors the ClimbSubsystem simulation pattern: both lift motors are modelled
     * as independent NEO DCMotorSims. Two hardstops are simulated:
     * <ul>
     *   <li>Ceiling at encoder = 0 (retracted hardstop) — stalls motor to produce a current spike
     *       that triggers {@link #isAtRetractedHardstop()} so homing can complete in simulation.</li>
     *   <li>Floor at {@code loweredPositionRotations} (extended hardstop) — stalls motor at the
     *       fully-lowered position so the intake rests without motor effort.</li>
     * </ul>
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        simulateLiftMotor(lift1SparkMaxSim, lift1MotorSim, dt, true);
        simulateLiftMotor(lift2SparkMaxSim, lift2MotorSim, dt, false);

        // Retrieve results
        simLift1Current = lift1SparkMaxSim.getMotorCurrent();
        simLift2Current = lift2SparkMaxSim.getMotorCurrent();
        simLift1Position = lift1MotorSim.getAngularPositionRotations();
        simLift2Position = lift2MotorSim.getAngularPositionRotations();
    }

    /**
     * Simulates a single lift motor with physical hardstops at both travel limits.
     *
     * @param sparkSim REV sim bridge for current/velocity
     * @param motorSim DCMotorSim physics model
     * @param dt       elapsed time since last tick (seconds)
     * @param isMotor1 used only to store simulated position into the correct field
     */
    private void simulateLiftMotor(SparkMaxSim sparkSim, DCMotorSim motorSim, double dt, boolean isMotor1) {
        double voltage = sparkSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        double currentPos = motorSim.getAngularPositionRotations();
        double floor = context.getLoweredPositionRotations();

        if (currentPos >= 0.0 && voltage > 0.0) {
            // At retracted ceiling hardstop — stall motor to produce current spike for homing
            motorSim.setState(VecBuilder.fill(0.0, 0.0));
            sparkSim.iterate(0.0, RobotController.getBatteryVoltage(), dt);
            if (isMotor1) simLift1Position = 0.0;
            else simLift2Position = 0.0;
        } else if (currentPos <= floor && voltage < 0.0) {
            // At extended floor hardstop — stall motor, intake rests on hardstop
            motorSim.setState(VecBuilder.fill(floor, 0.0));
            sparkSim.iterate(0.0, RobotController.getBatteryVoltage(), dt);
            if (isMotor1) simLift1Position = floor;
            else simLift2Position = floor;
        } else {
            motorSim.setInputVoltage(voltage);
            motorSim.update(dt);
            sparkSim.iterate(motorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
        }
    }
}
