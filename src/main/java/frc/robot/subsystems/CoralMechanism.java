package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralMechanism extends SubsystemBase {

    TalonSRX m_coralMotor1 = new TalonSRX(Constants.Port.m_CoralMtrRC);
    TalonSRX m_coralMotor2 = new TalonSRX(Constants.Port.m_CoralMtrLC);
    AnalogInput IRb = new AnalogInput(0);
    AnalogInput IRf = new AnalogInput(1);

    public CoralMechanism() {

    }

    public void CoralForward() {
        m_coralMotor1.set(ControlMode.PercentOutput, .60);
        m_coralMotor2.set(ControlMode.PercentOutput, -.60);
    }


    public void CoralBackward() {
        m_coralMotor1.set(ControlMode.PercentOutput, -.60);
        m_coralMotor2.set(ControlMode.PercentOutput, .60);
    }

    public void CoralTroph() {
        m_coralMotor1.set(ControlMode.PercentOutput, .95);
        m_coralMotor2.set(ControlMode.PercentOutput, -.25);
    }

    public void CoralStop() {
        m_coralMotor1.set(ControlMode.PercentOutput, 0);
        m_coralMotor2.set(ControlMode.PercentOutput, 0);
    }


    public Command CoralForwardCmd() {
        return this.runEnd(this::CoralForward, this::CoralStop);
    }

    public Command CoralTrophCmd() {
        return this.runEnd(this::CoralTroph, this::CoralStop);
    }

    public Command CoralBackwardCmd() {
        return this.runEnd(this::CoralBackward, this::CoralStop);
    }

    public Command CoralIntakeAutoCmd() {
        return this.runEnd(this::CoralForward, this::CoralStop).onlyWhile(() -> !IsCoralLoaded());
    }

    public Command CoralStopCmd() {
        return this.runOnce(this::CoralStop);
    }

    public int IrReadingf() {
        return IRf.getValue();
    }

    public int IrReadingb() {
        return IRb.getValue();
    }

    public boolean IsCoralLoaded() {
        return (IrReadingf() > 2200 && IrReadingb() > 1600);
    }

    public void periodic() {

    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Coral/IRf", () -> IrReadingf(), null);
        builder.addDoubleProperty("Coral/IRb", () -> IrReadingb(), null);
        builder.addBooleanProperty("Coral/CoralLoaded", () -> IsCoralLoaded(), null);
    }
}
