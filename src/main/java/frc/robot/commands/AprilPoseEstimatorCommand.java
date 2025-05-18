package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;

public class AprilPoseEstimatorCommand extends Command {
    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(.5, .5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(.5, .5);
    private static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(
            Units.degreesToRadians(400), Units.degreesToRadians(360));

    private final ProfiledPIDController m_xController = new ProfiledPIDController(.1, 0, 0.0, X_CONSTRAINTS);
    private final ProfiledPIDController m_yController = new ProfiledPIDController(.1, 0, 0.0, Y_CONSTRAINTS);
    private final ProfiledPIDController m_omegaController = new ProfiledPIDController(.1, 0, 0.0, OMEGA_CONSTRAINTS);

    private Drivetrain m_drivetrain;
    private Supplier<Pose2d> m_currentEstimatedPose;
    private Supplier<AprilTag> m_currentAprilTag;

    private Boolean m_isLeft;

    private double m_goalX;
    private double m_goalY;
    private double m_goalRot;

    public AprilPoseEstimatorCommand(Supplier<Pose2d> currentEstimatedPose, Supplier<AprilTag> currentAprilTag,
            boolean isLeft, Drivetrain drivetrainSubsystem) {
        m_currentEstimatedPose = currentEstimatedPose;
        m_currentAprilTag = currentAprilTag;
        m_isLeft = isLeft;
        m_drivetrain = drivetrainSubsystem;

        m_xController.setTolerance(0.1);
        m_yController.setTolerance(0.1);
        m_omegaController.setTolerance(Units.degreesToRadians(1));
        m_omegaController.enableContinuousInput(-Math.PI, Math.PI);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_drivetrain.setFieldRelativeEnable(false);
        AprilTag aprilTag = m_currentAprilTag.get();
        Pose2d goalPose = getGoalPose(aprilTag.ID);
        m_goalX = goalPose.getX();
        m_goalY = goalPose.getY();
        m_goalRot = goalPose.getRotation().getDegrees();

        if (m_goalX == -99999) {
            m_drivetrain.stopModules();
            return;
        }

        double xSpeed = 0;
        double ySpeed = 0;
        double rotSpeed = 0;
        Pose2d EstimatedPose = m_currentEstimatedPose.get();
        m_yController.setGoal(m_goalY);
        m_omegaController.setGoal(m_goalRot);
        m_xController.setGoal(m_goalX);

        xSpeed = m_xController.calculate(EstimatedPose.getX());
        if (m_xController.atGoal()) {
            xSpeed = 0;
        }

        ySpeed = m_yController.calculate(EstimatedPose.getY());
        if (m_yController.atGoal()) {
            ySpeed = 0;
        }

        rotSpeed = m_omegaController.calculate(EstimatedPose.getRotation().getDegrees());
        if (m_omegaController.atGoal()) {
            rotSpeed = 0;
        }

        m_drivetrain.drive(-xSpeed, -ySpeed, rotSpeed);

        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/EstimatePoseX",
                EstimatedPose.getX());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/EstimatePoseY",
                EstimatedPose.getY());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/EstimatePoseRot",
                EstimatedPose.getRotation().getDegrees());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/GoalPoseX",
                goalPose.getX());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/GoalPoseY",
                goalPose.getY());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/GoalPoseRot",
                goalPose.getRotation().getDegrees());
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/CalcVelX", xSpeed);
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/CalcVelY", ySpeed);
        SmartDashboard.putNumber(m_drivetrain.getName() + "/AprilAlignPoseEstimatorCommand/Command/CalcVelRot",
                rotSpeed);

    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.stopModules();
        m_drivetrain.setFieldRelativeEnable(true);
    }

    @Override
    public boolean isFinished() {
        return m_omegaController.atGoal() && m_xController.atGoal() && m_yController.atGoal();
    }

    public Pose2d getGoalPose(int currentAprilTagID) {
        Pose2d goalPose2d;
        switch (currentAprilTagID) {
            case 6 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(13.714, 2.868, new Rotation2d(120));
                } else {
                    goalPose2d = new Pose2d(13.930, 3.012, new Rotation2d(120));
                }
                break;
            case 7 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(14.373, 4.019, new Rotation2d(180));
                } else {
                    goalPose2d = new Pose2d(14.373, 4.199, new Rotation2d(180));
                }
                break;
            case 8 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(13.738, 5.158, new Rotation2d(-120));
                } else {
                    goalPose2d = new Pose2d(13.570, 5.266, new Rotation2d(-120));
                }
                break;
            case 9 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(14.373, 5.182, new Rotation2d(-60));
                } else {
                    goalPose2d = new Pose2d(12.263, 5.098, new Rotation2d(-60));
                }
                break;
            case 10 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(11.736, 4.019, new Rotation2d(0));
                } else {
                    goalPose2d = new Pose2d(11.736, 3.839, new Rotation2d(0));
                }
                break;
            case 11 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(12.407, 2.904, new Rotation2d(60));
                } else {
                    goalPose2d = new Pose2d(12.563, 2.796, new Rotation2d(60));
                }
                break;
            case 17 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.824, 2.904, new Rotation2d(60));
                } else {
                    goalPose2d = new Pose2d(3.992, 2.820, new Rotation2d(60));
                }
                break;
            case 18 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.165, 4.031, new Rotation2d(0));
                } else {
                    goalPose2d = new Pose2d(3.177, 3.815, new Rotation2d(0));
                }
                break;
            case 19 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.824, 5.170, new Rotation2d(-60));
                } else {
                    goalPose2d = new Pose2d(3.668, 5.086, new Rotation2d(-60));
                }
                break;
            case 20 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.155, 5.170, new Rotation2d(-60));
                } else {
                    goalPose2d = new Pose2d(5.023, 5.242, new Rotation2d(-60));
                }
                break;
            case 21 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.814, 4.019, new Rotation2d(180));
                } else {
                    goalPose2d = new Pose2d(5.814, 4.307, new Rotation2d(180));
                }
                break;
            case 22 :
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.119, 2.880, new Rotation2d(120));
                } else {
                    goalPose2d = new Pose2d(5.394, 3.024, new Rotation2d(120));
                }
                break;
            default :
                goalPose2d = new Pose2d(-99999, 0, new Rotation2d());
                break;
        }
        return goalPose2d;
    }
}
