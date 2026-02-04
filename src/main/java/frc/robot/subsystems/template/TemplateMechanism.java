package frc.robot.subsystems.template;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;

/**
 * A {@link Subsystem} template for a basic single-motor mechanism using Phoenix6 TalonFX.
 * This serves as a baseline reference for 2026 subsystem development.
 *
 * <p>Features demonstrated:
 * <ul>
 *   <li>Phoenix6 TalonFX motor controller integration</li>
 *   <li>Context-based configuration pattern</li>
 *   <li>Basic advance/reverse behavior with configurable speeds</li>
 *   <li>Command factory methods following WPILib best practices</li>
 * </ul>
 */
public class TemplateMechanism extends SubsystemBase {

    /** Mechanism state for telemetry visibility. */
    public enum State {
        STOPPED,
        ADVANCING,
        REVERSING
    }

    private static final String TELEMETRY_PREFIX = "Template";

    private final TemplateMechanismContext context;
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest;

    private State currentState = State.STOPPED;
    private double lastOutputPercent = 0.0;

    /**
     * Instantiates a new TemplateMechanism with default {@link TemplateMechanismContext}
     */
    public TemplateMechanism() {
        this(TemplateMechanismContext.defaults());
    }

    /**
     * Instantiates a new TemplateMechanism subsystem with the specified settings
     *
     * @param context The TemplateMechanismContext to apply to this instance
     */
    public TemplateMechanism(final TemplateMechanismContext context) {
        requireNonNull(context, "TemplateMechanismContext cannot be null");
        this.context = context;

        this.motor = new TalonFX(this.context.getMotorId(), this.context.getCanBus());
        this.dutyCycleRequest = new DutyCycleOut(0).withEnableFOC(true);

        configureMotor();
        initializeTelemetry();
    }

    /**
     * Initializes telemetry: registers for auto-capture and logs configuration.
     */
    private void initializeTelemetry() {
        Telemetry.registerSubsystem(TELEMETRY_PREFIX, this::captureTelemetry);

        // Log configuration once at startup
        Telemetry.event(TELEMETRY_PREFIX + "/Started", "MotorID=" + context.getMotorId());
        Telemetry.record(TELEMETRY_PREFIX + "/Config/MotorId", context.getMotorId(), TelemetryLevel.MATCH);
        Telemetry.record(TELEMETRY_PREFIX + "/Config/CanBus", context.getCanBus(), TelemetryLevel.LAB);
        Telemetry.record(TELEMETRY_PREFIX + "/Config/CurrentLimit", context.getCurrentLimit(), TelemetryLevel.LAB);
        Telemetry.record(TELEMETRY_PREFIX + "/Config/AdvanceSpeed", context.getAdvanceSpeed(), TelemetryLevel.LAB);
        Telemetry.record(TELEMETRY_PREFIX + "/Config/ReverseSpeed", context.getReverseSpeed(), TelemetryLevel.LAB);
    }

    /**
     * Captures telemetry data. Called automatically by Telemetry.periodic().
     *
     * @param prefix The telemetry key prefix (will be "Template")
     */
    private void captureTelemetry(String prefix) {
        // MATCH level - competition-critical data
        Telemetry.record(prefix + "/State", currentState.name(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Motor/OutputPercent", lastOutputPercent, TelemetryLevel.MATCH);
        Telemetry.record(
                prefix + "/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble(), TelemetryLevel.MATCH);

        // LAB level - tuning and diagnostics
        Telemetry.record(prefix + "/Motor/Temperature", motor.getDeviceTemp().getValueAsDouble(), TelemetryLevel.LAB);
        Telemetry.record(
                prefix + "/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble(), TelemetryLevel.LAB);
        Telemetry.record(
                prefix + "/Motor/SupplyVoltage", motor.getSupplyVoltage().getValueAsDouble(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/Motor/Velocity", motor.getVelocity().getValueAsDouble(), TelemetryLevel.LAB);

        // VERBOSE level - deep diagnostics
        Telemetry.record(prefix + "/Motor/Position", motor.getPosition().getValueAsDouble(), TelemetryLevel.VERBOSE);
        Telemetry.record(
                prefix + "/Motor/Acceleration", motor.getAcceleration().getValueAsDouble(), TelemetryLevel.VERBOSE);
    }

    /**
     * Configures the TalonFX motor with initial settings
     */
    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        // Set motor to brake mode by default
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        // Set current limits if desired
        config.CurrentLimits.SupplyCurrentLimit = this.context.getCurrentLimit();
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        // Apply configuration
        this.motor.getConfigurator().apply(config);
    }

    /**
     * Advances the motor forward at the configured speed.
     */
    private void advance() {
        if (currentState != State.ADVANCING) {
            Telemetry.event(TELEMETRY_PREFIX + "/Advancing", "speed=" + context.getAdvanceSpeed());
        }
        currentState = State.ADVANCING;
        lastOutputPercent = context.getAdvanceSpeed();
        this.motor.setControl(dutyCycleRequest.withOutput(lastOutputPercent));
    }

    /**
     * Reverses the motor backward at the configured speed.
     */
    private void reverse() {
        if (currentState != State.REVERSING) {
            Telemetry.event(TELEMETRY_PREFIX + "/Reversing", "speed=" + context.getReverseSpeed());
        }
        currentState = State.REVERSING;
        lastOutputPercent = context.getReverseSpeed();
        this.motor.setControl(dutyCycleRequest.withOutput(lastOutputPercent));
    }

    /**
     * Stops the motor.
     */
    private void stop() {
        if (currentState != State.STOPPED) {
            Telemetry.event(TELEMETRY_PREFIX + "/Stopped", "from=" + currentState.name());
        }
        currentState = State.STOPPED;
        lastOutputPercent = 0.0;
        this.motor.setControl(dutyCycleRequest.withOutput(0));
    }

    /**
     * Obtains a "runEnd" Command which will advance the motor on each iteration.
     * Interruption of the Command will trigger stopping of the motor.
     *
     * @return The Command
     */
    public Command getAdvanceCommand() {
        return this.runEnd(this::advance, this::stop);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the motor on each iteration.
     * Interruption of the Command will trigger stopping of the motor.
     *
     * @return The Command
     */
    public Command getReverseCommand() {
        return this.runEnd(this::reverse, this::stop);
    }

    /**
     * Obtains a Command which stops the motor
     *
     * @return The Command
     */
    public Command getStopCommand() {
        return this.runOnce(this::stop);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any periodic status updates or telemetry here
    }
}
