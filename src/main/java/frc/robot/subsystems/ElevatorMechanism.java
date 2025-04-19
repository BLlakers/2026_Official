package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

enum ElevatorState {
    Down,
    Troph,
    L2,
    L3,
    L4
}

public class ElevatorMechanism extends SubsystemBase {
    public double m_elevatorSpeed;
    public static double Down = 0;
    public static double Troph = 2.5;
    public static double L2 = 5.8;
    public static double AlgaeGround = 6.2;
    public static double AlgaeL3 = 10.2;
    public static double L3 = 13;
    public static double AlgaeL4 = 17.6;
    public static double L4 = 24.5;
    public static boolean IsMoving;
    private static double kDt = 0.02;
    private static double kMaxVelocity = .3;
    private static double kMaxAcceleration = 0.3;
    private static double kP = 0.2;
    private static double kI = 0.0;
    private static double kD = 0.0;
    private static double kS = .275;
    private static double kG = 1;
    private static double kV = .455;
    private double marginOfError = 1;
    private double elevatorPositionConversionFactor = 1.6 * Math.PI; // 1.6 * Math.PI = Distance per rotation
    private double elevatorVelocityConversionFactor = 1;
    private double desiredPos;
    private ElevatorState Estate = ElevatorState.Down;
    private double elevDecelerateOffset = 5.6;
    //public double position;
    public double elevatorPosition;
    private ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, ELEVATOR_CONSTRAINTS);
    private static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(140), Units.feetToMeters(125));
    private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(kS, kG, kV);

    //A motor to rotate up and down
    private SparkMax m_ElevatorMotor = new SparkMax(Constants.Port.m_ElevatorMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private SparkMax mFollower = new SparkMax(Constants.Port.m_Follower, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private DigitalInput m_ElevatorLimitSwitchTop = new DigitalInput(6);
    private DigitalInput m_ElevatorLimitSwitchBottom = new DigitalInput(7);
    public Boolean AtBottom = true;
    private SparkMaxConfig m_ElevatorConfig = new SparkMaxConfig();
    private SparkMaxConfig mFollowerConfig = new SparkMaxConfig();

    public ElevatorMechanism() {
        m_ElevatorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(1.0, 0, 0);
        m_ElevatorConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake);
        m_ElevatorConfig.alternateEncoder //TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
                .positionConversionFactor(elevatorPositionConversionFactor)
                .velocityConversionFactor(elevatorVelocityConversionFactor)
                .countsPerRevolution(8192);
        m_ElevatorMotor.configure(m_ElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        mFollowerConfig.follow(Constants.Port.m_ElevatorMtrC, true);

        m_ElevatorMotor.configure(m_ElevatorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        mFollower.configure(mFollowerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        pid.setTolerance(.16);
        ResetPosition();

    }

    public void ElevatorMotorUp() {
        if (!m_ElevatorLimitSwitchTop.get()) {
            m_ElevatorMotor.set(.15);
        } else {
            m_ElevatorMotor.set(0);
        }

    }

    public void ResetPosition() {
        m_ElevatorMotor.getAlternateEncoder().setPosition(0);
    }

    public Command ResetPositionCMD() {
        return this.runOnce(this::ResetPosition);
    }

    public void ElevatorMotorDown() {
        m_ElevatorMotor.set(-.05);
    }

    public boolean ElevatorLimitSwitchTop() {
        return m_ElevatorLimitSwitchTop.get();
    }

    public boolean ElevatorLimitSwitchBottom() {
        return m_ElevatorLimitSwitchBottom.get();
    }

    public void ElevatorMotorStop() {
        m_ElevatorMotor.set(0);
    }

    public void ElevatorMove(double d) {
        m_ElevatorMotor.set(d);
    }

    public void ElevatorMoveV(double d) {
        m_ElevatorMotor.setVoltage(d);
    }

    public double getElevatorEncoderPos() {
        return m_ElevatorMotor.getAlternateEncoder().getPosition();
    }

    public double getElevatorDecelerateRatio() {
        return 1 - ((getElevatorEncoderPos()) / (L4 + elevDecelerateOffset));
    }


    public boolean ElevatorAtPos() {
        return getElevatorEncoderPos() > 23.7;
    }

    public Command ElevatorUpLimitCmd() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop).until(() -> ElevatorLimitSwitchTop());
    }

    public Command ElevatorDownLimitCmd() {
        return this.runEnd(this::ElevatorMotorDown, this::ElevatorMotorStop).until(() -> ElevatorLimitSwitchBottom());
    }

    public Command ElevatorUpCmd() {
        return this.runEnd(this::ElevatorMotorUp, this::ElevatorMotorStop);
    }

    public Command ElevatorDownCmd() {
        return this.runEnd(this::ElevatorMotorDown, this::ElevatorMotorStop);
    }

    public Command ElevatorStopCmd() {
        return this.runOnce(this::ElevatorMotorStop);
    }

    public void desiredPosSet(double s) {
        desiredPos = s;
    }

    public void MoveDesiredPosUp() {
        if (Estate == ElevatorState.Down) {
            Estate = ElevatorState.Troph;
        } else if (Estate == ElevatorState.Troph) {
            Estate = ElevatorState.L2;
        } else if (Estate == ElevatorState.L2) {
            Estate = ElevatorState.L3;
        } else if (Estate == ElevatorState.L3) {
            Estate = ElevatorState.L4;
        } else if (Estate == ElevatorState.L4) {
            Estate = ElevatorState.L4;
        }

    }

    public void MoveDesiredPosDown() {
        if (Estate == ElevatorState.L4) {
            Estate = ElevatorState.L3;
        } else if (Estate == ElevatorState.L3) {
            Estate = ElevatorState.L2;
        } else if (Estate == ElevatorState.L2) {
            Estate = ElevatorState.Troph;
        } else if (Estate == ElevatorState.Troph) {
            Estate = ElevatorState.Down;
        } else if (Estate == ElevatorState.Down) {
            Estate = ElevatorState.Down;
        }
    }

    public void ChangeDesiredPos() {
        if (Estate == ElevatorState.L4) {
            desiredPos = L4;
        } else if (Estate == ElevatorState.L3) {
            desiredPos = L3;

        } else if (Estate == ElevatorState.L2) {
            desiredPos = L2;
        } else if (Estate == ElevatorState.Troph) {
            desiredPos = Troph;
        } else if (Estate == ElevatorState.Down) {
            desiredPos = Down;
        }
    }


    public Command MovePosUp() {

        return runOnce(() -> MoveDesiredPosUp()).andThen(() -> ChangeDesiredPos());
    }

    public Command MovePosDown() {

        return runOnce(() -> MoveDesiredPosDown()).andThen(() -> ChangeDesiredPos());
    }

    public double desiredPosGet() {
        return desiredPos;
    }

    public void ResetElevatorEnc() {
        if (ElevatorLimitSwitchBottom() == true) {
            m_ElevatorMotor.getAlternateEncoder().setPosition(0);
            AtBottom = true;
        } else {
            AtBottom = false;
        }
    }

    public ElevatorState getEstate() {
        return this.Estate;
    }

    public void initElevatorPID() {
        pid.reset(getElevatorEncoderPos());
    }

    public void setElevatorPIDPos(double desiredPos) {
        elevatorPosition = desiredPos;
    }

    public void pid(
            double position) {
        pid.setGoal(position);
        m_elevatorSpeed = ((18) * pid.calculate(getElevatorEncoderPos()) + .7) /*kG/*m_feedforward.calculate(pid.getSetpoint().velocity)*/;

        if (ElevatorLimitSwitchTop() && m_elevatorSpeed > 0) {
            m_elevatorSpeed = 0;
        }

        if (ElevatorLimitSwitchBottom() && m_elevatorSpeed < 0) {
            m_elevatorSpeed = 0;
        }

        ElevatorMoveV(m_elevatorSpeed);
    }

    public boolean atPIDGoal() {
        return pid.atGoal();
    }


    @Override
    public void periodic() {
        pid(elevatorPosition);
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorSpeedInVolts", () -> m_elevatorSpeed, null);
        builder.addDoubleProperty(getName() + "ElevatorCommand/Command/elevatorDesirePIDPos", () -> elevatorPosition, null);
        builder.addDoubleProperty("Elevator/Position", () -> getElevatorEncoderPos(), null);
        builder.addBooleanProperty("Elevator/LimitSwitchTop", this::ElevatorLimitSwitchTop, null);
        builder.addBooleanProperty("Elevator/LimitSwitchBottom", this::ElevatorLimitSwitchBottom, null);
        builder.addBooleanProperty("Elevator/AtPos", this::ElevatorAtPos, null);
        builder.addBooleanProperty("Elevator/AtPIDGoal", this::atPIDGoal, null);
        builder.addDoubleProperty("Elevator/DecelerateRatio", () -> getElevatorDecelerateRatio(), null);
    }
}
