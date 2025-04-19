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
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeMechanism extends SubsystemBase {
    private ProfiledPIDController pid = new ProfiledPIDController(.75, 0, 0, ALGAE_CONSTRAINTS);
    private static final TrapezoidProfile.Constraints ALGAE_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(10), Units.feetToMeters(8));
    public static double PosDown = -1.03;
    public static double PosUp = -0.14;
    public static double PosMiddle = -0.75;
    public static double PosGround = -1.165;
    public static double irDistance = 100;

    private double m_AlgaeSpeed;
    public static double posUpreg = 0;
    public static double posMiddlereg = 4.5;
    public static double posDownreg = 9;
    public double algaePosition;
    AnalogInput AlgaeIR = new AnalogInput(2);
    private static final double IRVALUE = 2400;
    public static double GEAR_RATIO = 75;
    double algaePositionConversionFactor =
            360 / AlgaeMechanism.GEAR_RATIO; // revolutions -> radians
    private double algaeVelocityConversionFactor = 1;
    public static boolean AUTORunning;
    //private DigitalInput m_AlgaeLimitSwitchTop = new DigitalInput(7);
    //A motor to rotate up and down
    private SparkMax m_AlgaeMotor = new SparkMax(Constants.Algae.m_AlgaeMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    private SparkMaxConfig m_AlgaeConfig = new SparkMaxConfig();


    public AlgaeMechanism() {
        m_AlgaeConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.0, 0, 0);
        m_AlgaeConfig
                .inverted(false)
                .idleMode(IdleMode.kCoast);
        m_AlgaeConfig.alternateEncoder //TODO MAKE SURE TO USE RIGHT TYPE OF ENCODER WHEN DOING CONFIGS!
                .positionConversionFactor(algaePositionConversionFactor)
                .velocityConversionFactor(algaeVelocityConversionFactor)
                .countsPerRevolution(8192);
        pid.setTolerance(0.035);
        m_AlgaeMotor.configure(m_AlgaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        ResetPosition();
    }

    public void AlgaeForward() {
        m_AlgaeMotor.set(.1);
    }

    public void AlgaeBackward() {
        m_AlgaeMotor.set(-.1);
    }

    public boolean IntakeFowardIR() {
        if (AlgaeIR.getValue() >= IRVALUE) {
            System.out.println("TRUE: " + AlgaeIR.getValue() + ">= " + IRVALUE);
            return true;
        } else {
            return false;
        }
    }

    public void AlgaeStop() {
        m_AlgaeMotor.set(0);
    }

    public void AlgaeMove(double m) {
        m_AlgaeMotor.set(-m);
    }

    public double getAlgaePos() {
        return m_AlgaeMotor.getAlternateEncoder().getPosition();
    }

    public void ResetPosition() {
        m_AlgaeMotor.getAlternateEncoder().setPosition(0);
    }

    public void setAlgaePIDPosition(double desiredPos) {
        algaePosition = desiredPos;
    }

    public void AlgaePID(double desPosition) {
        // if (AlgaeIR() <= irDistance){
        //     pid.setGoal(PosMiddle);
        // }else{
        //     pid.setGoal(desPosition);
        // }
        pid.setGoal(desPosition);
        m_AlgaeSpeed = pid.calculate(getAlgaePos());
        if (pid.atGoal()) {
            m_AlgaeSpeed = 0;
        }
        AlgaeMove(m_AlgaeSpeed);
    }

    public void initAlgaePID() {
        pid.reset(getAlgaePos());
    }

    public boolean CheckAlgaePID() {
        return pid.atGoal();
    }

    @Override
    public void periodic() {
        if (!AUTORunning) {
            if (IntakeFowardIR()) {
                algaePosition = PosMiddle;
                AlgaePID(PosMiddle);
            } else {
                AlgaePID(algaePosition);
            }
        }
        // AlgaePID(algaePosition);
    }

    public Command AlgaeForwardCmd() {
        return this.runEnd(this::AlgaeForward, this::AlgaeStop);
    }

    public Command AlgaeBackwardCmd() {
        return this.runEnd(this::AlgaeBackward, this::AlgaeStop);
    }

    public Command AlgaeStopCmd() {
        return this.runOnce(this::AlgaeStop);
    }

    public Command resetAlgae() {
        return this.runOnce(() -> ResetPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty(this.getName() + "/Algae/Position/EncoderPos", () -> getAlgaePos(), null);
        builder.addDoubleProperty(this.getName() + "/Algae/Position/Goal", () -> pid.getGoal().position, null);
        builder.addBooleanProperty(this.getName() + "/Algae/Position/atGoal", () -> CheckAlgaePID(), null);
        builder.addDoubleProperty(this.getName() + "/Algae/Position/Speed", () -> m_AlgaeSpeed, null);
        builder.addBooleanProperty(this.getName() + "/Intake/IRGOOD", () -> IntakeFowardIR(), null);
        builder.addDoubleProperty(this.getName() + "/Intake/IRValue", () -> AlgaeIR.getValue(), null);
        builder.addBooleanProperty(this.getName() + "/Algae/AutoRunning", () -> AUTORunning, null);

    }
}
