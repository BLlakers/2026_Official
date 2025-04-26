package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.PIDSettings;

import static java.util.Objects.requireNonNull;

/**
 * A {@link Subsystem} implementation responsible for the operation of the climbing mechanism
 */
public class ClimbMechanism extends SubsystemBase {

    private final SparkMax climbMotor;

    private final DigitalInput climbMagSwitch;

    private final ClimbMechanismSettings settings;

    /**
     * Instantiates a new AlgaeIntake with default {@link ClimbMechanismSettings}
     */
    public ClimbMechanism() {
        this(ClimbMechanismSettings.defaults());
    }

    /**
     * Instantiates a new ClimbMechanism subsystem with the specified settings
     * @param settings The ClimbMechanismSettings to apply to this instance
     */
    public ClimbMechanism(final ClimbMechanismSettings settings){
        requireNonNull(settings, "ClimbMechanismSettings cannot be null");
        this.settings = settings;
        this.climbMotor = new SparkMax(this.settings.getClimbMotorChannel(), MotorType.kBrushless);
        this.climbMotor.configure(this.assembleClimbMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.climbMagSwitch = new DigitalInput(this.settings.getClimbMagSwitchChannel());
    }

    /**
     * Prepares and configures a {@link SparkMaxConfig} to be applied to this mechanism's climb motor
     * @return The SparkMaxConfig
     */
    private SparkMaxConfig assembleClimbMotorConfig(){
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(this.settings.getClimbPositionConversionFactor())
                .velocityConversionFactor(this.settings.getClimbVelocityConversionFactor());
        PIDSettings pidSettings = this.settings.getClimbControllerPIDSettings();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());
        return config;
    }

    /**
     * Advances the climb motor by the specified speed advancement increment
     */
    private void advanceClimbMotor() {
        this.climbMotor.set(this.settings.getAdvanceIncrement());
    }

    /**
     * Reverses the climb motor by the specified speed reversing increment
     */
    private void reverseClimbMotor() {
        this.climbMotor.set(this.settings.getReverseIncrement());
    }

    /**
     * Stops the progress of the climb motor
     */
    private void stopClimbMotor() {
        this.climbMotor.set(0);
    }

    /**
     * Indicates that the climb mechanism is in the "down" position
     * @return Indication
     */
    private boolean isClimbDown() {
        return !this.climbMagSwitch.get();
    }

    /**
     * Obtains a "runEnd" Command which will advance the climb mechanism motor on each iteration.
     * Interruption of the Command will trigger stopping of the climb mechanism motor.
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getAdvanceClimbCommand() {
        return this.runEnd(this::advanceClimbMotor, this::stopClimbMotor);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the climb mechanism motor on each iteration.
     * Interruption of the Command will trigger stopping of the climb mechanism motor.
     * @return The Command
     */
    // TODO: Remove if unused
    public Command getReverseClimbCommand() {
        return this.runEnd(this::reverseClimbMotor, this::stopClimbMotor);
    }

    /**
     * Obtains a "runEnd" Command which will reverse the climb mechanism motor until it is
     * fully down per {@link ClimbMechanism#isClimbDown()} and which point it will stop.
     * Interruption of the Command will trigger stopping of the climb mechanism motor.
     * @return The Command
     */
    public Command getFullyReverseClimbCommand() {
        return this.runEnd(this::reverseClimbMotor, this::stopClimbMotor).until(this::isClimbDown).finallyDo(this::stopClimbMotor);
    }

    /**
     * Obtains a "runOnce" Command which will stop the climb mechanism motor.
     * @return The Command
     */
    public Command getStopClimbCommand() {
        return this.runOnce(this::stopClimbMotor);
    }

    /**
     * Obtains the climb motor's {@link RelativeEncoder} position
     * @return The position
     */
    private double getCurrentClimbPosition() {
        return this.climbMotor.getAlternateEncoder().getPosition();
    }

    /**
     * {@link Sendable#initSendable(SendableBuilder)} implementation
     * @param builder The sendable builder
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Climb/Position", this::getCurrentClimbPosition, null);
    }
}
