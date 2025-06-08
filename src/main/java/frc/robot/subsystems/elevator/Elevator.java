package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;

public class Elevator extends SubsystemBase {

    private final ElevatorSettings settings;

    private double elevatorSpeed;

    private double elevatorPosition;

    // A motor to rotate up and down
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorFollowerMotor;
    private final ProfiledPIDController elevatorController;
    private final DigitalInput elevatorLimitSwitchTop;
    private final DigitalInput elevatorLimitSwitchBottom;

    public Elevator() {
        this(ElevatorSettings.defaults());
    }

    public Elevator(final ElevatorSettings settings) {
        this.settings = settings;

        this.elevatorLimitSwitchTop = new DigitalInput(this.settings.getElevatorLimitSwitchTopChannel());

        this.elevatorLimitSwitchBottom = new DigitalInput(this.settings.getElevatorLimitSwitchBottomChannel());

        this.elevatorMotor = new SparkMax(this.settings.getElevatorMotorChannel(), MotorType.kBrushless);

        this.elevatorMotor.configure(this.assembleElevatorMotorConfig(), ResetMode.kNoResetSafeParameters,
                PersistMode.kPersistParameters);

        this.elevatorFollowerMotor = new SparkMax(this.settings.getElevatorFollowerMotorChannel(),
                MotorType.kBrushless);
        this.elevatorFollowerMotor.configure(this.assembleElevatorFollowerMotorConfig(),
                ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        PIDSettings pidSettings = this.settings.getElevatorControllerPIDSettings();
        this.elevatorController = new ProfiledPIDController(pidSettings.p(), pidSettings.i(), pidSettings.d(),
                this.settings.getElevatorConstraints());
        this.elevatorController.setTolerance(this.settings.getElevatorControllerTolerance());

        this.resetPosition();
    }

    private void advanceElevatorMotorDown() {
        this.elevatorMotor.set(this.settings.getReverseIncrement());
    }

    private SparkMaxConfig assembleElevatorMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        PIDSettings pidSettings = this.settings.getFeedbackSensorPIDSettings();
        config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).pid(pidSettings.p(),
                pidSettings.i(), pidSettings.d());
        config.alternateEncoder.positionConversionFactor(this.settings.getElevatorPositionConversionFactor())
                .velocityConversionFactor(this.settings.getElevatorVelocityConversionFactor())
                .countsPerRevolution(this.settings.getCountsPerRevolution());
        return config;
    }

    private SparkMaxConfig assembleElevatorFollowerMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(Constants.Port.elevatorMotorChannel, true);
        return config;
    }

    private void setElevatorMotorUp() {
        if (!elevatorLimitSwitchTop.get()) {
            this.elevatorMotor.set(this.settings.getAdvanceIncrement());
        } else {
            this.elevatorMotor.set(0);
        }
    }

    private void resetPosition() {
        this.elevatorMotor.getAlternateEncoder().setPosition(0);
    }

    private boolean isElevatorLimitSwitchTop() {
        return elevatorLimitSwitchTop.get();
    }

    private boolean isElevatorLimitSwitchBottom() {
        return elevatorLimitSwitchBottom.get();
    }

    private void setElevatorMotorMoveVoltage(double voltage) {
        this.elevatorMotor.setVoltage(voltage);
    }

    private double getElevatorMotorEncoderPosition() {
        return this.elevatorMotor.getAlternateEncoder().getPosition();
    }

    private void updatePIDController(double position) {
        this.elevatorController.setGoal(position);
        this.elevatorSpeed = ((this.settings.getElevatorSpeedBoostFactor())
                * this.elevatorController.calculate(this.getElevatorMotorEncoderPosition())
                + this.settings.getElevatorSpeedMotorPositionAdjustment());

        if ((this.isElevatorLimitSwitchTop() && this.elevatorSpeed > 0)
                || (this.isElevatorLimitSwitchBottom() && this.elevatorSpeed < 0)) {
            this.elevatorSpeed = 0;
        }
        this.setElevatorMotorMoveVoltage(this.elevatorSpeed);
    }

    private boolean isAtPIDGoal() {
        return this.elevatorController.atGoal();
    }

    public void stopElevatorMotor() {
        this.elevatorMotor.set(0);
    }

    public double getElevatorDecelerateRatio() {
        return 1 - ((this.getElevatorMotorEncoderPosition())
                / (this.settings.getLevel4Position() + this.settings.getElevatorDecelerationOffset()));
    }

    public boolean isElevatorAtPosition() {
        return this.getElevatorMotorEncoderPosition() > this.settings.getElevatorCeiling();
    }

    public Command getResetPositionCommand() {
        return this.runOnce(this::resetPosition);
    }

    public Command getElevatorUpLimitCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor).until(this::isElevatorLimitSwitchTop);
    }

    public Command getElevatorDownLimitCommand() {
        return this.runEnd(this::advanceElevatorMotorDown, this::stopElevatorMotor)
                .until(this::isElevatorLimitSwitchBottom);
    }

    public Command getElevatorUpCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor);
    }

    public void initElevatorPID() {
        this.elevatorController.reset(this.getElevatorMotorEncoderPosition());
    }

    public void setElevatorPIDPosition(double position) {
        this.elevatorPosition = position;
    }

    @Override
    public void periodic() {
        updatePIDController(this.elevatorPosition);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorSpeedInVolts", () -> this.elevatorSpeed,
                null);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorDesirePIDPos",
                () -> this.elevatorPosition, null);
        builder.addDoubleProperty("Elevator/Position", this::getElevatorMotorEncoderPosition, null);
        builder.addBooleanProperty("Elevator/LimitSwitchTop", this::isElevatorLimitSwitchTop, null);
        builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::isElevatorLimitSwitchBottom, null);
        builder.addBooleanProperty("Elevator/AtPos", this::isElevatorAtPosition, null);
        builder.addBooleanProperty("Elevator/AtPIDGoal", this::isAtPIDGoal, null);
        builder.addDoubleProperty("Elevator/DecelerateRatio", this::getElevatorDecelerateRatio, null);
    }
}
