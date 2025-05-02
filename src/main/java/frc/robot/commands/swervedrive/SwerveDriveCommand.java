package frc.robot.commands.swervedrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class SwerveDriveCommand extends Command {

    // TODO: Consider a configuration location for this adjustment constant
    private static final double MAX_DRIVE_SPEED_ADJUSTMENT = 0.95;

    // TODO: Consider a configuration location for this adjustment constant
    private static final double TURN_MAX_SPEED_ADJUSTMENT = 0.50;

    private final ControllerDelegate controllerDelegate;

    private final Drivetrain drivetrain;

    private final double driveMaxSpeed;

    private final double turnMaxSpeed;

    public SwerveDriveCommand(
            final ControllerDelegate controllerDelegate,
            final Drivetrain drivetrain) {
        this.controllerDelegate = controllerDelegate;
        this.drivetrain = drivetrain;
        this.driveMaxSpeed = MAX_DRIVE_SPEED_ADJUSTMENT * this.drivetrain.getMaxSpeed();
        this.turnMaxSpeed = TURN_MAX_SPEED_ADJUSTMENT * this.drivetrain.getMaxTurnAngularSpeed();
        addRequirements(this.drivetrain);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double x, y, rot;
        double acceleration = this.controllerDelegate.getAcceleration();
        double leftX = this.controllerDelegate.getLeftX();
        double leftY = this.controllerDelegate.getLeftY();
        double rightX = this.controllerDelegate.getRightX();
        double elevatorDecelerationRatio = this.controllerDelegate.getElevatorDecelerateRatio();

        // Finds the X Value of the Left Stick on the Controller and Takes Care of
        // Joystick Drift
        x = MathUtil.applyDeadband(-leftX, Constants.Controller.deadzone);

        // Finds the Y Value of the Left Stick on the Controller and Takes Care of
        // Joystick Drift
        y = MathUtil.applyDeadband(-leftY, Constants.Controller.deadzone);

        // Finds the X Value of the Right Stick on the Controller and Takes Care of
        // Joystick Drift
        rot = MathUtil.applyDeadband(-rightX, Constants.Controller.deadzone);
        double xSpeed;
        double ySpeed;
        double rotSpeed = rot * this.turnMaxSpeed * elevatorDecelerationRatio;

        if (this.controllerDelegate.getDriver().getLabel().equals(Constants.DriverLabels.ASA)) {
            xSpeed = y * this.driveMaxSpeed * elevatorDecelerationRatio;
            ySpeed = x * this.driveMaxSpeed * elevatorDecelerationRatio;
        } else {
            xSpeed = y * this.driveMaxSpeed * acceleration * elevatorDecelerationRatio;
            ySpeed = x * this.driveMaxSpeed * acceleration * elevatorDecelerationRatio;
        }


        // TODO: x and y are assigned to and never used again. I'm assuming this is just an oversight in the code.
        // To be clear, this is an issue because x and y and on the local scope of this method... did the author intend
        // for their values to be normalized and retained across executions... because they're not, as they are stack-level
        // variables so commenting this out...
//        double normalizingFactor = Math.hypot(x, y);
//        if (normalizingFactor > 0) {
//            x /= normalizingFactor;
//            y /= normalizingFactor;
//        }


        if (this.controllerDelegate.isHalfSpeed()) {
            xSpeed /= 2;
            ySpeed /= 2;
            rotSpeed /= 2;
        }

        SmartDashboard.putNumber("DriveTrain/Controller/Command/X Speed", xSpeed);
        SmartDashboard.putNumber("DriveTrain/Controller/Command/Y Speed", ySpeed);
        SmartDashboard.putNumber("DriveTrain/Controller/Command/Rot Speed", rotSpeed);

        drivetrain.drive(xSpeed, ySpeed, rotSpeed);
    }
}
