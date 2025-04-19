package frc.robot.commands;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorMechanism;

public class ElevatorPID extends Command {
    private Double position;
    private ElevatorMechanism elevator;


    public ElevatorPID(ElevatorMechanism e, Double p) {

        elevator = e;
        position = p;

        addRequirements(elevator);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        elevator.initElevatorPID();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        elevator.setElevatorPIDPos(position);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.ElevatorMotorStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
