package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Servo extends SubsystemBase {
    PWM servo = new PWM(9);

    public Servo() {

    }

    public void ServoForward() {
        servo.setPosition(.8);
    }

    public void ServoBackward() {
        servo.setPosition(0);
    }

    public void ServoMiddle() {
        servo.setPosition(.5);
    }

    public Command ServoForwardCommand() {
        return this.runEnd(this::ServoForward, this::ServoMiddle);
    }

    public Command ServoBackwardCommand() {
        return this.runEnd(this::ServoBackward, this::ServoMiddle);
    }
}
