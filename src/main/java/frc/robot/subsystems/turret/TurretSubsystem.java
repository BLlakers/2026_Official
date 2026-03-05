package frc.robot.subsystems.turret;

import static java.util.Objects.requireNonNull;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.function.DoubleSupplier;

/**
 * Turret subsystem — rotates the shooter assembly horizontally to aim at the hub.
 *
 * <h2>Mechanism Overview</h2>
 * <p>A single NEO drives a 9:1 stacked gearbox, then a 44→74t stage, then a 30→120t stage
 * (total: ≈60.55:1). The shooter assembly rotates asymmetrically: 200° left (CCW) and 100°
 * right (CW) from home. The SparkMax built-in encoder tracks motor rotations; angle at the
 * turret is computed by dividing by the total gear ratio.
 *
 * <h2>Encoder Convention</h2>
 * <ul>
 *   <li>0 rotations = home (turret aimed straight forward)</li>
 *   <li>Positive encoder count = counterclockwise (left) rotation</li>
 *   <li>Negative encoder count = clockwise (right) rotation</li>
 * </ul>
 * This matches the sign convention used by {@code TurretTracker}, so
 * {@code TurretTracker.getTurretAngleDegrees()} feeds directly into
 * {@link #getTrackCommand(DoubleSupplier)} without sign inversion.
 *
 * <h2>Homing</h2>
 * <p>The turret must be manually rotated to the home position (facing straight forward) before
 * powering on. On boot, the SparkMax encoder is zeroed automatically via {@link #resetEncoder()}.
 * The {@link #getResetTurretRotationCommand()} command can re-zero the encoder at any time if
 * the turret is repositioned after boot.
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

        // Zero the encoder on boot. The turret must be physically positioned at home
        // (facing straight forward) before powering on.
        resetEncoder();

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
        // No positionConversionFactor needed: the SparkMax firmware normalises the NEO's
        // 42 counts/rev internally, so getPosition() already returns motor rotations.
        // getCurrentAngleDegrees() converts to turret degrees via the gear ratio.
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
     * Zeros the SparkMax encoder, declaring the current physical turret position as home (0°).
     *
     * <p>Called automatically on boot. Can also be triggered via
     * {@link #getResetTurretRotationCommand()} if the turret is repositioned after power-on.
     */
    private void resetEncoder() {
        turretMotor.getEncoder().setPosition(0.0);
    }

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
        // Soft limit — stop at max left boundary
        if (getCurrentAngleDegrees() >= context.getMaxLeftDegrees()) {
            turretMotor.set(0);
            return;
        }
        turretMotor.set(context.getTurretJogSpeed());
    }

    private void jogRight() {
        // Soft limit — stop at max right boundary
        if (getCurrentAngleDegrees() <= -context.getMaxRightDegrees()) {
            turretMotor.set(0);
            return;
        }
        turretMotor.set(-context.getTurretJogSpeed());
    }

    private void stop() {
        turretMotor.set(0);
    }

    // -------------------------------------------------------------------------
    // Command factories
    // -------------------------------------------------------------------------

    /**
     * Reset command — zeros the SparkMax encoder, declaring the current turret position as home.
     *
     * <p>Use this if the encoder drifted or the turret was manually repositioned after boot.
     *
     * @return Command that resets the turret encoder to zero
     */
    public Command getResetTurretRotationCommand() {
        return this.runOnce(this::resetEncoder).withName("Turret.ResetEncoder");
    }

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
                    // Clamp target to the asymmetric soft limits before computing error.
                    // TurretTracker already clamps, but this is a safety second layer in case
                    // the command is used with any other supplier.
                    double targetDegrees = MathUtil.clamp(
                            targetAngleDegreesSupplier.getAsDouble(),
                            -context.getMaxRightDegrees(),
                            context.getMaxLeftDegrees());
                    double errorDegrees = targetDegrees - getCurrentAngleDegrees();
                    double output =
                            MathUtil.clamp(TRACKING_KP * errorDegrees, -TRACKING_MAX_OUTPUT, TRACKING_MAX_OUTPUT);
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
        Telemetry.publish(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/AngleDeg", angleDegrees, TelemetryLevel.MATCH);
        Telemetry.publish(prefix + "/OutputPercent", turretMotor.getAppliedOutput(), TelemetryLevel.MATCH);

        // LAB level
        Telemetry.publish(prefix + "/Current", turretMotor.getOutputCurrent(), TelemetryLevel.LAB);
        Telemetry.publish(prefix + "/EncoderRotations", turretMotor.getEncoder().getPosition(), TelemetryLevel.LAB);
    }

    // -------------------------------------------------------------------------
    // Periodic
    // -------------------------------------------------------------------------

    @Override
    public void periodic() {
        // Telemetry is captured by the registered subsystem callback via Telemetry.periodic()
    }
}
