package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
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
    public static final double ELEVATOR_CEILING = 23.7;
    public static double ALGAE_L3_POSITION = 10.2;
    public static double L3_POSITION = 13;
    public static double ALGAE_L4_POSITION = 17.6;
    public static double L4_POSITION = 24.5;
    public static final double MOTOR_ADVANCE_DOWN_INCREMENT = -.05;
    public static final int COUNTS_PER_REVOLUTION = 8192;
    private static final double ELEVATOR_DECELERATE_OFFSET = 5.6;
    private static final double ELEVATOR_POSITION_CONVERSION_FACTOR = 1.6 * Math.PI; // 1.6 * Math.PI = Distance per rotation
    private static final double ELEVATOR_VELOCITY_CONVERSION_FACTOR = 1;
    private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(140), Units.feetToMeters(125));
    private static final ProfiledPIDController PID = new ProfiledPIDController(0.2, 0.0, 0.0, ELEVATOR_CONSTRAINTS);

    private double elevatorSpeed;
    private double elevatorPosition;

    //A motor to rotate up and down
    private static final SparkMax ELEVATOR_MOTOR = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private static final SparkMax ELEVATOR_FOLLOWER_MOTOR = new SparkMax(Constants.Port.m_Follower, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private final DigitalInput elevatorLimitSwitchTop = new DigitalInput(6);
    private final DigitalInput elevatorLimitSwitchBottom = new DigitalInput(7);

    public ElevatorMechanism() {
        ELEVATOR_MOTOR.configure(this.assembleElevatorMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        ELEVATOR_FOLLOWER_MOTOR.configure(this.assembleElevatorFollowerMotorConfig(), ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        PID.setTolerance(.16);
        ResetPosition();

    }

    private void advanceElevatorMotorDown() {
        ELEVATOR_MOTOR.set(MOTOR_ADVANCE_DOWN_INCREMENT);
    }

    private SparkMaxConfig assembleElevatorMotorConfig(){
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .p(1.0)
                .i(0)
                .d(0);
        config.alternateEncoder //TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
                .positionConversionFactor(ELEVATOR_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(ELEVATOR_VELOCITY_CONVERSION_FACTOR)
                .countsPerRevolution(COUNTS_PER_REVOLUTION);
        return config;
    }

    private SparkMaxConfig assembleElevatorFollowerMotorConfig(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.follow(Constants.Port.m_ElevatorMtrC, true);
        return config;
    }

    public void ElevatorMotorUp() {
        if (!elevatorLimitSwitchTop.get()) {
            ELEVATOR_MOTOR.set(.15);
        } else {
            ELEVATOR_MOTOR.set(0);
        }
    }

    public void ResetPosition() {
        ELEVATOR_MOTOR.getAlternateEncoder().setPosition(0);
    }

    public boolean ElevatorLimitSwitchTop() {
        return elevatorLimitSwitchTop.get();
    }

    public boolean ElevatorLimitSwitchBottom() {
        return elevatorLimitSwitchBottom.get();
    }

    public void ElevatorMotorStop() {
        ELEVATOR_MOTOR.set(0);
    }

    public void ElevatorMoveV(double d) {
        ELEVATOR_MOTOR.setVoltage(d);
    }

    public double getElevatorEncoderPos() {
        return ELEVATOR_MOTOR.getAlternateEncoder().getPosition();
    }

    public double getElevatorDecelerateRatio() {
        return 1 - ((getElevatorEncoderPos()) / (L4_POSITION + ELEVATOR_DECELERATE_OFFSET));
    }

    public boolean isElevatorAtPosition() {
        return getElevatorEncoderPos() > ELEVATOR_CEILING;
    }

    public Command ResetPositionCMD() {
        return this.runOnce(this::ResetPosition);
    }

    public Command getElevatorUpLimitCommand() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop).until(this::ElevatorLimitSwitchTop);
    }

    public Command getElevatorDownLimitCommand() {
        return this.runEnd(this::advanceElevatorMotorDown, this::ElevatorMotorStop).until(this::ElevatorLimitSwitchBottom);
    }

    public Command getElevatorUpCommand() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop);
    }

    public void initElevatorPID() {
        PID.reset(getElevatorEncoderPos());
    }

    public void setElevatorPIDPos(double desiredPos) {
        elevatorPosition = desiredPos;
    }

    public void pid(double position) {
        PID.setGoal(position);
        elevatorSpeed = ((18) * PID.calculate(getElevatorEncoderPos()) + .7);

        if (ElevatorLimitSwitchTop() && elevatorSpeed > 0) {
            elevatorSpeed = 0;
        }

        if (ElevatorLimitSwitchBottom() && elevatorSpeed < 0) {
            elevatorSpeed = 0;
        }

        ElevatorMoveV(elevatorSpeed);
    }

    public boolean atPIDGoal() {
        return PID.atGoal();
    }


    @Override
    public void periodic() {
        pid(elevatorPosition);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorSpeedInVolts", () -> elevatorSpeed, null);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorDesirePIDPos", () -> elevatorPosition, null);
        builder.addDoubleProperty("Elevator/Position", this::getElevatorEncoderPos, null);
        builder.addBooleanProperty("Elevator/LimitSwitchTop", this::ElevatorLimitSwitchTop, null);
        builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::ElevatorLimitSwitchBottom, null);
        builder.addBooleanProperty("Elevator/AtPos", this::isElevatorAtPosition, null);
        builder.addBooleanProperty("Elevator/AtPIDGoal", this::atPIDGoal, null);
        builder.addDoubleProperty("Elevator/DecelerateRatio", this::getElevatorDecelerateRatio, null);
    }
}
