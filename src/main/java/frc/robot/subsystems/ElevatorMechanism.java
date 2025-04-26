package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorMechanism extends SubsystemBase {

    public static final double DOWN_POSITION = 0;
    public static final double TROPH_POSITION = 2.5;
    public static final double L2_POSITION = 5.8;
    public static final double ALGAE_GROUND_POSITION = 6.2;
    public static final double ALGAE_L3_POSITION = 10.2;
    public static final double L3_POSITION = 13;
    public static final double ALGAE_L4_POSITION = 17.6;
    public static final double L4_POSITION = 24.5;
    public static final double MOTOR_ADVANCE_DOWN_INCREMENT = -.05;
    public static final int COUNTS_PER_REVOLUTION = 8192;
    private static final double ELEVATOR_CEILING = 23.7;
    private static final double ELEVATOR_DECELERATE_OFFSET = 5.6;
    private static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.6 * Math.PI; // 1.6 * Math.PI = Distance per rotation
    private static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = 1;
    private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(140), Units.feetToMeters(125));

    private double elevatorSpeed;
    private double elevatorPosition;

    //A motor to rotate up and down
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorFollowerMotor;
    private final ProfiledPIDController pidController;
    private final DigitalInput elevatorLimitSwitchTop = new DigitalInput(6);
    private final DigitalInput elevatorLimitSwitchBottom = new DigitalInput(7);

    public ElevatorMechanism() {
        this.elevatorMotor = new SparkMax(Constants.Port.elevatorMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        this.elevatorMotor.configure(this.assembleElevatorMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.elevatorFollowerMotor = new SparkMax(Constants.Port.elevatorFollowerMotorChannel, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
        this.elevatorFollowerMotor.configure(this.assembleElevatorFollowerMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        this.pidController = new ProfiledPIDController(0.2, 0.0, 0.0, ELEVATOR_CONSTRAINTS);
        this.pidController.setTolerance(.16);
        resetPosition();
    }

    private void advanceElevatorMotorDown() {
        elevatorMotor.set(ElevatorMechanism.MOTOR_ADVANCE_DOWN_INCREMENT);
    }

    private SparkMaxConfig assembleElevatorMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder).pid(1.0, 0, 0);
        config.alternateEncoder
                .positionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ELEVATOR_VELOCITY_CONVERSION_FACTOR)
                .countsPerRevolution(ElevatorMechanism.COUNTS_PER_REVOLUTION);
        return config;
    }

    private SparkMaxConfig assembleElevatorFollowerMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(Constants.Port.elevatorMotorChannel, true);
        return config;
    }

    private void setElevatorMotorUp() {
        if (!elevatorLimitSwitchTop.get()) {
            this.elevatorMotor.set(.15);
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
        this.pidController.setGoal(position);
        this.elevatorSpeed = ((18) * this.pidController.calculate(this.getElevatorMotorEncoderPosition()) + .7);
        if ((this.isElevatorLimitSwitchTop() && this.elevatorSpeed > 0)
                || (this.isElevatorLimitSwitchBottom() && this.elevatorSpeed < 0)) {
            this.elevatorSpeed = 0;
        }
        this.setElevatorMotorMoveVoltage(this.elevatorSpeed);
    }

    private boolean isAtPIDGoal() {
        return this.pidController.atGoal();
    }

    public void stopElevatorMotor() {
        this.elevatorMotor.set(0);
    }

    public double getElevatorDecelerateRatio() {
        return 1 - ((this.getElevatorMotorEncoderPosition()) / (ElevatorMechanism.L4_POSITION + ELEVATOR_DECELERATE_OFFSET));
    }

    public boolean isElevatorAtPosition() {
        return this.getElevatorMotorEncoderPosition() > ELEVATOR_CEILING;
    }

    public Command getResetPositionCommand() {
        return this.runOnce(this::resetPosition);
    }

    public Command getElevatorUpLimitCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor).until(this::isElevatorLimitSwitchTop);
    }

    public Command getElevatorDownLimitCommand() {
        return this.runEnd(this::advanceElevatorMotorDown, this::stopElevatorMotor).until(this::isElevatorLimitSwitchBottom);
    }

    public Command getElevatorUpCommand() {
        return this.runEnd(this::setElevatorMotorUp, this::stopElevatorMotor);
    }

    public void initElevatorPID() {
        this.pidController.reset(this.getElevatorMotorEncoderPosition());
    }

    public void setElevatorPIDPosition(double position) {
        this.elevatorPosition = position;
    }

    @Override
    public void periodic() {
        updatePIDController(elevatorPosition);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorSpeedInVolts", () -> elevatorSpeed, null);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorDesirePIDPos", () -> elevatorPosition, null);
        builder.addDoubleProperty("Elevator/Position", this::getElevatorMotorEncoderPosition, null);
        builder.addBooleanProperty("Elevator/LimitSwitchTop", this::isElevatorLimitSwitchTop, null);
        builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::isElevatorLimitSwitchBottom, null);
        builder.addBooleanProperty("Elevator/AtPos", this::isElevatorAtPosition, null);
        builder.addBooleanProperty("Elevator/AtPIDGoal", this::isAtPIDGoal, null);
        builder.addDoubleProperty("Elevator/DecelerateRatio", this::getElevatorDecelerateRatio, null);
    }
}
