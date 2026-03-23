package frc.robot.subsystems.relay;

import static edu.wpi.first.math.system.plant.LinearSystemId.createDCMotorSystem;
import static java.util.Objects.requireNonNull;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * Relay subsystem — conveys fuel balls from the intake to the indexer.
 *
 * <h2>Mechanism Overview</h2>
 * <p>Six roller bars fitted with soft grabbing pads span the width of the robot between the
 * intake exit and the indexer entrance. All bars are belt-coupled to the innermost bar, which
 * is directly driven by a single NEO (10:1) on a SparkMax. There is no position sensing —
 * the relay is open-loop only.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → rollers convey balls toward the indexer</li>
 *   <li>Negative output → rollers reverse to clear jams</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — positive output must move balls toward the indexer.
 *       See {@link RelaySubsystemContext#isRelayMotorInverted()}</li>
 *   <li>Tune {@code relaySpeed} for reliable ball handoff to the indexer</li>
 *   <li>Tune {@code relayReverseSpeed} for effective jam clearing</li>
 * </ul>
 */
public class RelaySubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Relay";

    /** Operating states of the relay mechanism. */
    public enum State {
        /** Motor stopped. */
        IDLE,
        /** Rollers spinning to convey balls toward the indexer. */
        RUNNING,
        /** Rollers spinning in reverse to clear jams. */
        REVERSING,
    }

    private final RelaySubsystemContext context;

    // Motor
    private final SparkMax relayMotor; // NEO

    private State currentState = State.IDLE;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim relayMotorSim;
    private SparkMaxSim relaySparkMaxSim;
    private double lastSimTime = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates a RelaySubsystem with default context.
     */
    public RelaySubsystem() {
        this(RelaySubsystemContext.defaults());
    }

    /**
     * Instantiates a RelaySubsystem with the specified context.
     *
     * @param context The RelaySubsystemContext to apply to this instance
     */
    public RelaySubsystem(final RelaySubsystemContext context) {
        requireNonNull(context, "RelaySubsystemContext cannot be null");
        this.context = context;

        this.relayMotor = new SparkMax(context.getRelayMotorId(), MotorType.kBrushless);

        configureMotor();

        if (RobotBase.isSimulation()) {
            this.relaySparkMaxSim = new SparkMaxSim(relayMotor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> plant = createDCMotorSystem(DCMotor.getNEO(1), 0.001, context.getRelayGearRatio());
            this.relayMotorSim = new DCMotorSim(plant, DCMotor.getNEO(1));
            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(context.getRelayCurrentLimit());
        config.idleMode(IdleMode.kBrake); // Coast so rollers don't snap-stop and jam balls
        config.inverted(context.isRelayMotorInverted());
        relayMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // State helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    private double getRelayPosition() {
        return relayMotor.getEncoder().getPosition();
    }

    /**
     * Returns the current operating state of the relay.
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
        if (getRelayPosition() <= 7.0) {
            relayMotor.set(context.getRelaySpeed());
        } else {
            stop();
        }
    }

    private void runReverse() {
        if (getRelayPosition() >= 0.0) {
            relayMotor.set(context.getRelayReverseSpeed());
        } else {
            stop();
        }
    }

    private void stop() {
        relayMotor.set(0);
    }

    private boolean relayAtTop() {
        return getRelayPosition() >= 4.5;
    }

    private boolean relayAtMiddle() {
        return getRelayPosition() <= 3.1;
    }

    private boolean relayAtBottom() {
        return getRelayPosition() <= 0.3;
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Run command — spins the relay rollers to convey balls toward the indexer.
     *
     * <p>Held-button command: rollers spin while the button is held, stop on release.
     *
     * @return Command that runs the relay while held
     */
    public Command getRunCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.RUNNING);
                            runForward();
                        },
                        () -> {
                            stop();
                            if (currentState == State.RUNNING) setState(State.IDLE);
                        })
                .withName("Relay.Run");
    }

    /**
     * Reverse command — spins the relay rollers in reverse to clear jams.
     *
     * <p>Held-button command: rollers spin in reverse while held, stop on release.
     *
     * @return Command that reverses the relay while held
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
                .withName("Relay.Reverse");
    }

    /**
     * Stop command — stops the relay motor (emergency / safe shutdown).
     *
     * @return Command that stops the relay
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {
                    stop();
                    setState(State.IDLE);
                })
                .withName("Relay.Stop");
    }

    /**
     * Agitate command — spins the relay rollers in reverse to clear jams.
     *
     * <p>Held-button command: rollers spin in reverse while held, stop on release.
     *
     * @return Command that agitates the relay while held
     */
    public Command getAgitateCommand() {
        return Commands.repeatingSequence(
                        getRunCommand().until(() -> relayAtTop()),
                        getReverseCommand().until(() -> relayAtMiddle()))
                .finallyDo(interrupted -> {
                    if (interrupted) {
                        getHommingCommand().schedule();
                    }
                })
                .withName("Relay.Agitate");
    }

    public Command getHommingCommand() {
        return getReverseCommand().until(() -> relayAtBottom()).withName("Relay.Homing");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "RelayMotorID=" + context.getRelayMotorId());
    }

    private void captureTelemetry(String prefix) {
        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/OutputPercent", relayMotor.getAppliedOutput(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/Position", getRelayPosition(), TelemetryLevel.MATCH);

        // LAB level
        Telemetry.record(prefix + "/Current", relayMotor.getOutputCurrent(), TelemetryLevel.LAB);

        // VERBOSE level
        if (RobotBase.isSimulation()) {
            Telemetry.record(prefix + "/VelocityRPM", relayMotorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
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
     * Advances simulated relay motor physics each tick.
     *
     * <p>The relay has no hardstops — this simply propagates the applied voltage through
     * the NEO DCMotorSim to produce realistic velocity and current readings for telemetry.
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        double voltage = relaySparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        relayMotorSim.setInputVoltage(voltage);
        relayMotorSim.update(dt);
        relaySparkMaxSim.iterate(relayMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
    }
}
