package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import frc.robot.support.limelight.LimelightHelpers;

public class LimelightAlignCommand extends Command {

    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =
            new TrapezoidProfile.Constraints(Units.degreesToRadians(400), Units.degreesToRadians(360));

    private final ProfiledPIDController m_omegaController = new ProfiledPIDController(2, 0, 0.0, OMEGA_CONSTRAINTS);

    private Drivetrain m_drivetrain;

    public LimelightAlignCommand(Drivetrain drivetrainSubsystem) {
        m_drivetrain = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        int numTargets = LimelightHelpers.getTargetCount("limelight-front");
         SmartDashboard.putNumber(m_drivetrain.getName() + "/LimelightAlign/numTargets", numTargets);
        if (numTargets
                == 0) { // is valid if > 0: we update our current estimate of where the april tag is
            // relative to the robot
            m_drivetrain.stopModules();
            return;
        }

        double rotSpeed = 0;

        m_omegaController.setGoal(0);
        double limelightReading = LimelightHelpers.getTX("limelight-front");
        rotSpeed = m_omegaController.calculate(limelightReading);

        if (m_omegaController.atGoal()) {
            rotSpeed = 0;
        }

        m_drivetrain.drive(0, 0, rotSpeed);

        SmartDashboard.putNumber(m_drivetrain.getName() + "/LimelightAlign/rotSpeed", rotSpeed);
        SmartDashboard.putNumber(m_drivetrain.getName() + "/LimelightAlign/limelightReading", limelightReading);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopModules();
    }

    @Override
    public boolean isFinished() {
        return m_omegaController.atGoal();
    }
}
