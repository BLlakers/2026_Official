package frc.robot.subsystems.indexer;

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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * Indexer subsystem — queues fuel balls and advances them to the shooter.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The indexer is a vertical tower structure that receives fuel balls from the relay. Two
 * grabbing wheels mounted on a shared driveshaft spin in opposite directions, pulling the next
 * ball into a void at the center of the tower. Balls queue sequentially inside the chamber and
 * are pushed upward toward the shooter as each one is consumed. The driveshaft is driven by a
 * single NEO (1:1) via belt on a SparkMax.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → wheels pull balls into chamber and advance toward shooter</li>
 *   <li>Negative output → wheels reverse to clear jams</li>
 * </ul>
 *
 * <h2>Coordination</h2>
 * <p>The indexer is commanded in unison with the relay and shooter via a
 * {@code Commands.parallel()} group bound to the manipulator right trigger. It is not
 * intended to run independently during normal match play.
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>Confirm motor inversion — positive output must advance balls toward the shooter.
 *       See {@link IndexerSubsystemContext#isIndexerMotorInverted()}</li>
 *   <li>Tune {@code indexerSpeed} — balance between reliable advancement and ball control</li>
 *   <li>Tune {@code indexerReverseSpeed} for effective jam clearing</li>
 * </ul>
 */
public class IndexerSubsystem extends SubsystemBase {

    private static final String TELEMETRY_PREFIX = "Indexer";

    /** Operating states of the indexer mechanism. */
    public enum State {
        /** Motor stopped. */
        IDLE,
        /** Wheels spinning to pull balls into chamber and advance toward shooter. */
        INDEXING,
        /** Wheels spinning in reverse to clear jams. */
        REVERSING,
    }

    private final IndexerSubsystemContext context;

    // Motor
    private final SparkMax indexerMotor; // NEO

    private State currentState = State.IDLE;

    // -------------------------------------------------------------------------
    // Simulation fields (only initialized when RobotBase.isSimulation())
    // -------------------------------------------------------------------------

    private DCMotorSim indexerMotorSim;
    private SparkMaxSim indexerSparkMaxSim;
    private double lastSimTime = 0.0;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /**
     * Instantiates an IndexerSubsystem with default context.
     */
    public IndexerSubsystem() {
        this(IndexerSubsystemContext.defaults());
    }

    /**
     * Instantiates an IndexerSubsystem with the specified context.
     *
     * @param context The IndexerSubsystemContext to apply to this instance
     */
    public IndexerSubsystem(final IndexerSubsystemContext context) {
        requireNonNull(context, "IndexerSubsystemContext cannot be null");
        this.context = context;

        this.indexerMotor = new SparkMax(context.getIndexerMotorId(), MotorType.kBrushless);

        configureMotor();

        if (RobotBase.isSimulation()) {
            this.indexerSparkMaxSim = new SparkMaxSim(indexerMotor, DCMotor.getNEO(1));
            LinearSystem<N2, N1, N2> plant =
                    createDCMotorSystem(DCMotor.getNEO(1), 0.001, context.getIndexerGearRatio());
            this.indexerMotorSim = new DCMotorSim(plant, DCMotor.getNEO(1));
            this.lastSimTime = Timer.getFPGATimestamp();
        }

        initializeTelemetry();
    }

    // -------------------------------------------------------------------------
    // Configuration
    // -------------------------------------------------------------------------

    private void configureMotor() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(context.getIndexerCurrentLimit());
        config.idleMode(IdleMode.kCoast); // Coast so balls don't snap-stop and jam in chamber
        config.inverted(context.isIndexerMotorInverted());
        indexerMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // -------------------------------------------------------------------------
    // State helpers
    // -------------------------------------------------------------------------

    private void setState(State state) {
        this.currentState = state;
    }

    /**
     * Returns the current operating state of the indexer.
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
        indexerMotor.set(context.getIndexerSpeed());
    }

    private void runReverse() {
        indexerMotor.set(context.getIndexerReverseSpeed());
    }

    private void stop() {
        indexerMotor.set(0);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Index command — spins the grabbing wheels to pull balls into the chamber and advance
     * them toward the shooter.
     *
     * <p>Intended to run in parallel with {@code RelaySubsystem.getRunCommand()} and
     * the shooter command via a {@code Commands.parallel()} group on the manipulator right trigger.
     * Held-button: runs while held, stops on release.
     *
     * @return Command that runs the indexer while held
     */
    public Command getIndexCommand() {
        return this.runEnd(
                        () -> {
                            setState(State.INDEXING);
                            runForward();
                        },
                        () -> {
                            stop();
                            if (currentState == State.INDEXING) setState(State.IDLE);
                        })
                .withName("Indexer.Index");
    }

    /**
     * Reverse command — spins the grabbing wheels in reverse to clear jams.
     *
     * <p>Intended to run in parallel with {@code RelaySubsystem.getReverseCommand()} via a
     * {@code Commands.parallel()} group on the manipulator right bumper.
     * Held-button: runs while held, stops on release.
     *
     * @return Command that reverses the indexer while held
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
                .withName("Indexer.Reverse");
    }

    /**
     * Stop command — stops the indexer motor (emergency / safe shutdown).
     *
     * @return Command that stops the indexer
     */
    public Command getStopCommand() {
        return this.runOnce(() -> {
                    stop();
                    setState(State.IDLE);
                })
                .withName("Indexer.Stop");
    }

    // -------------------------------------------------------------------------
    // Telemetry
    // -------------------------------------------------------------------------

    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "IndexerMotorID=" + context.getIndexerMotorId());
    }

    private void captureTelemetry(String prefix) {
        // MATCH level
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/OutputPercent", indexerMotor.getAppliedOutput(), TelemetryLevel.MATCH);

        // LAB level
        Telemetry.record(prefix + "/Current", indexerMotor.getOutputCurrent(), TelemetryLevel.LAB);

        // VERBOSE level
        if (RobotBase.isSimulation()) {
            Telemetry.record(prefix + "/VelocityRPM", indexerMotorSim.getAngularVelocityRPM(), TelemetryLevel.VERBOSE);
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
     * Advances simulated indexer motor physics each tick.
     *
     * <p>The indexer has no hardstops — this simply propagates the applied voltage through
     * the NEO DCMotorSim to produce realistic velocity and current readings for telemetry.
     */
    @Override
    public void simulationPeriodic() {
        double now = Timer.getFPGATimestamp();
        double dt = now - lastSimTime;
        lastSimTime = now;

        double voltage = indexerSparkMaxSim.getAppliedOutput() * RobotController.getBatteryVoltage();
        indexerMotorSim.setInputVoltage(voltage);
        indexerMotorSim.update(dt);
        indexerSparkMaxSim.iterate(indexerMotorSim.getAngularVelocityRPM(), RobotController.getBatteryVoltage(), dt);
    }
}
