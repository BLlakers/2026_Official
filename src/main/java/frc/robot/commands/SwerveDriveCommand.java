package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drivetrain.Drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class SwerveDriveCommand extends Command {
    private DoubleSupplier m_leftY;
    private DoubleSupplier m_leftX;
    private DoubleSupplier m_rightX;
    private DoubleSupplier m_AccelerateRT;
    private Drivetrain m_Drivetrain;
    private String Driver;
    private BooleanSupplier m_RunHalfSpeed;
    private DoubleSupplier m_ElevatorDecelerate;

    private static final double kDriveMaxSpeed = 0.95 * Drivetrain.MAX_SPEED;
    private static final double kTurnMaxSpeed = 0.5 * Drivetrain.MAX_TURN_ANGULAR_SPEED;

    public SwerveDriveCommand(
            DoubleSupplier _leftY,
            DoubleSupplier _leftX,
            DoubleSupplier _rightX,
            DoubleSupplier _AccelerateRT,
            Drivetrain _dTrain) {
        m_leftY = _leftY;
        m_leftX = _leftX;
        m_rightX = _rightX;
        m_Drivetrain = _dTrain;
        m_AccelerateRT = _AccelerateRT;
        m_RunHalfSpeed = () -> false;
        m_ElevatorDecelerate = () -> 1.0;
        Driver = "Ben";
        addRequirements(m_Drivetrain);
    }

    public SwerveDriveCommand(
            DoubleSupplier _leftY,
            DoubleSupplier _leftX,
            DoubleSupplier _rightX,
            DoubleSupplier _AccelerateRT,
            Drivetrain _dTrain,
            BooleanSupplier _halfSpeedCondition,
            String d) {
        m_leftY = _leftY;
        m_leftX = _leftX;
        m_rightX = _rightX;
        m_Drivetrain = _dTrain;
        m_AccelerateRT = _AccelerateRT;
        m_RunHalfSpeed = _halfSpeedCondition;
        m_ElevatorDecelerate = () -> 1.0;
        addRequirements(m_Drivetrain);
        Driver = d;
    }

    public SwerveDriveCommand(
            DoubleSupplier _leftY,
            DoubleSupplier _leftX,
            DoubleSupplier _rightX,
            DoubleSupplier _AccelerateRT,
            DoubleSupplier _ElevatorDecelerate,
            Drivetrain _dTrain,
            BooleanSupplier _halfSpeedCondition,
            String d) {
        m_leftY = _leftY;
        m_leftX = _leftX;
        m_rightX = _rightX;
        m_Drivetrain = _dTrain;
        m_AccelerateRT = _AccelerateRT;
        m_RunHalfSpeed = _halfSpeedCondition;
        m_ElevatorDecelerate = _ElevatorDecelerate;
        Driver = d;
        addRequirements(m_Drivetrain);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double RT, x, y, rot, Elev;
        double AccelerateRT = m_AccelerateRT.getAsDouble();
        double leftX = m_leftX.getAsDouble();
        double leftY = m_leftY.getAsDouble();
        double rightX = m_rightX.getAsDouble();
        double DecelerateElev = m_ElevatorDecelerate.getAsDouble();

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
        double rotSpeed;
        Elev = DecelerateElev;
        RT = m_AccelerateRT.getAsDouble();
        if (Driver == "Asa") {
            xSpeed = y * SwerveDriveCommand.kDriveMaxSpeed * Elev;
            ySpeed = x * SwerveDriveCommand.kDriveMaxSpeed * Elev;
            rotSpeed = rot * SwerveDriveCommand.kTurnMaxSpeed * Elev;
        } else {
            xSpeed = y * SwerveDriveCommand.kDriveMaxSpeed * RT * Elev;
            ySpeed = x * SwerveDriveCommand.kDriveMaxSpeed * RT * Elev;
            rotSpeed = rot * SwerveDriveCommand.kTurnMaxSpeed * Elev;
        }


        double normalizingFactor = Math.hypot(x, y);
        if (normalizingFactor > 0) {
            x /= normalizingFactor;
            y /= normalizingFactor;
        }


        if (m_RunHalfSpeed.getAsBoolean() == true) {
            xSpeed /= 2;
            ySpeed /= 2;
            rotSpeed /= 2;
        }

        SmartDashboard.putNumber("DriveTrain/Controller/Command/X Speed", xSpeed);
        SmartDashboard.putNumber("DriveTrain/Controller/Command/Y Speed", ySpeed);

        m_Drivetrain.drive(xSpeed, ySpeed, rotSpeed);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
