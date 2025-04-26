package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.Supplier;

public class PathFindToPoseCommand extends Command {
    Supplier<Pose2d> currentPose2d;
    Boolean side;
    Drivetrain driveTrain;

    public void init() {
    }

    public PathFindToPoseCommand(Supplier<Pose2d> CurrentPose, Boolean Left, Drivetrain d) {
        currentPose2d = CurrentPose;
        side = Left;
        driveTrain = d;
    }

    @Override
    public void execute() {
        Pose2d RoboPose = currentPose2d.get();
        Pose2d goalPosition;

        if (side) {
            goalPosition = RoboPose.nearest(Constants.Poses.PositionsLeftBlue);
        } else {
            goalPosition = RoboPose.nearest(Constants.Poses.PositionsRightBlue);
        }
        System.out.println(goalPosition);

        AutoBuilder.pathfindToPose(goalPosition, RobotContainer.SPEED_CONSTRAINTS, 0.0);

    }
}
