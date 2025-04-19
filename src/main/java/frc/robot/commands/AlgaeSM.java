package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeMechanism;

public class AlgaeSM extends Command {
    private ProfiledPIDController pidThroughBore = new ProfiledPIDController(70, 0, 0, ALGAE_CONSTRAINTS);
    private static final TrapezoidProfile.Constraints ALGAE_CONSTRAINTS = new TrapezoidProfile.Constraints(Units.feetToMeters(10), Units.feetToMeters(8));
    private AlgaeMechanism m_algaeMech;
    private double position;

    public AlgaeSM(AlgaeMechanism algaeMech, Double pos) {
        position = pos;
        m_algaeMech = algaeMech;
        addRequirements(m_algaeMech);
    }


    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        pidThroughBore.reset(m_algaeMech.getAlgaePos());
        pidThroughBore.setTolerance(0.0005);

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double m_AlgaeSpeed;
        pidThroughBore.setGoal(position);
        m_AlgaeSpeed = pidThroughBore.calculate(m_algaeMech.getAlgaePos());
        if (pidThroughBore.atGoal()) {
            m_AlgaeSpeed = 0;
        }
        //return pidThroughBore.calculate(getAlgaeEncoderPosThroughBore());

        m_algaeMech.AlgaeMove(-m_AlgaeSpeed);
    }

    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // mAlgaeIntake.IntakeStop();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return true;
        // return pidThroughBore.atGoal() || m_algaeMech.AlgaeIR() <= 100;
    }
}
