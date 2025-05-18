package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// PMW(Pulse Width Modulation)
public class Servo extends SubsystemBase {

    private final ServoSettings settings;

    private final PWM servo;

    public Servo() {
        this(ServoSettings.defaults());
    }

    public Servo(final ServoSettings settings) {
        this.settings = settings;
        this.servo = new PWM(this.settings.getChannel());
    }

    public void forwardPosition() {
        servo.setPosition(this.settings.getForwardPosition());
    }

    public void backwardPosition() {
        servo.setPosition(this.settings.getBackwardPosition());
    }

    public void middlePosition() {
        servo.setPosition(this.settings.getMiddlePosition());
    }

    public Command getForwardPositionCommand() {
        return this.runEnd(this::forwardPosition, this::middlePosition);
    }

    public Command getBackwardCommand() {
        return this.runEnd(this::backwardPosition, this::middlePosition);
    }
}
