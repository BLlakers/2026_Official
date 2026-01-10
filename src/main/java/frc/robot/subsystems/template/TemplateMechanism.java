package frc.robot.subsystems.template;

import static java.util.Objects.requireNonNull;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    private final TemplateMechanismContext context;
    private final TalonFX motor;
    private final DutyCycleOut dutyCycleRequest;

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
     * Advances the motor forward at the configured speed
     */
    private void advance() {
        this.motor.setControl(dutyCycleRequest.withOutput(this.context.getAdvanceSpeed()));
    }

    /**
     * Reverses the motor backward at the configured speed
     */
    private void reverse() {
        this.motor.setControl(dutyCycleRequest.withOutput(this.context.getReverseSpeed()));
    }

    /**
     * Stops the motor
     */
    private void stop() {
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
