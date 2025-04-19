package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
    private TalonSRX m_IntakeMotor = new TalonSRX(Constants.Algae.m_IntakeMtrC);

    public AlgaeIntake() {
    }


    public void IntakeForward() {
        m_IntakeMotor.set(ControlMode.PercentOutput, .75);
    }

    public void IntakeBackward() {
        m_IntakeMotor.set(ControlMode.PercentOutput, -.85);
    }

    public void IntakeStop() {
        m_IntakeMotor.set(ControlMode.PercentOutput, 0);
    }

    public Command IntakeForwardCmd() {
        return this.runEnd(this::IntakeForward, this::IntakeStop);
    }


    public Command IntakeBackwardCmd() {
        return this.runEnd(this::IntakeBackward, this::IntakeStop);
    }

    public Command IntakeBackwardOnceCmd() {
        return this.runOnce(this::IntakeBackward);
    }

    public Command IntakeStopCmd() {
        return this.runOnce(this::IntakeStop);
    }

    public Command IntakeForwardOnceCmd() {
        return this.runOnce(this::IntakeForward);
    }

    public Command RunIntake() {
        return this.runEnd(this::IntakeForward, this::IntakeStop);/* .onlyWhile(()-> IntakeFowardIR() == false)*/
    }

    @Override
    public void initSendable(SendableBuilder builder) {

        super.initSendable(builder);
    }
}
