package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbMechanism extends SubsystemBase {
    private double climbPositionConversionFactor = 1;
    private double climbVelocityConversionFactor = 1;

    //A motor to rotate up and down
    SparkMax m_ClimbMotor = new SparkMax(Constants.Port.m_ClimbMtrC, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    SparkMaxConfig m_ClimbConfig = new SparkMaxConfig();

    RelativeEncoder m_ClimbMotorEncoder = m_ClimbMotor.getEncoder();

    private DigitalInput climbMagSwitch = new DigitalInput(Constants.Port.climbMagSwitchDIOC);

    public ClimbMechanism() {
        m_ClimbConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        m_ClimbConfig.encoder
                .positionConversionFactor(climbPositionConversionFactor)
                .velocityConversionFactor(climbVelocityConversionFactor);
        m_ClimbConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
                .pid(1.0, 0, 0);
    }

    public void WindStringForward() {
        m_ClimbMotor.set(.75);
    }

    public void WindStringBackward() {
        m_ClimbMotor.set(-.75);
    }

    public void WindStop() {
        m_ClimbMotor.set(0);
    }

    public boolean ClimbIsDown() {
        return !climbMagSwitch.get();
    }

    public Command WindForwardCmd() {
        return this.runEnd(this::WindStringForward, this::WindStop);
    }

    public Command WindBackwardCmd() {
        return this.runEnd(this::WindStringBackward, this::WindStop);
    }

    public Command WindDownCmd() {
        return this.runEnd(this::WindStringBackward, this::WindStop).until(this::ClimbIsDown).finallyDo(this::WindStop);
    }

    public Command WindStopCmd() {
        return this.runOnce(this::WindStop);
    }

    public double getClimbEncoderPos() {
        return m_ClimbMotor.getAlternateEncoder().getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Climb/Position", () -> getClimbEncoderPos(), null);
    }
}
