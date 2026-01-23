package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.swervedrive.ControllerDelegate;
import frc.robot.commands.swervedrive.SwerveDriveCommand;
import frc.robot.subsystems.LedStrand;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainContext;
import frc.robot.subsystems.fuel.FuelSubsystem;
import frc.robot.subsystems.fuel.FuelSubsystemContext;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemContext;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    private final LedStrand ledStrand = new LedStrand();

    private final DrivetrainContext drivetrainContext = DrivetrainContext.defaults();

    private final Drivetrain driveTrain = new Drivetrain(drivetrainContext);

    private final FuelSubsystem fuelSubsystem = new FuelSubsystem(FuelSubsystemContext.defaults());

    private final VisionSubsystem visionSubsystem = new VisionSubsystem(
            VisionSubsystemContext.builder().enablePhotonCameraSimStreams(true).build(),
            driveTrain,
            driveTrain::addVisionMeasurement);

    private final Command resetPoseAuto =
            Commands.runOnce(() -> this.driveTrain.resetOdometry(this.currentPath.get(0)), this.driveTrain);

    /**
     * Creates buttons and controller for: - the driver controller (port 0) - the manipulator controller (port 1) - the
     * debug controller (port 2)
     */
    private final CommandXboxController driverController =
            new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER_CHANNEL);

    private final CommandXboxController manipController =
            new CommandXboxController(Constants.Controller.MANIPULATION_CONTROLLER_CHANNEL);

    private final CommandXboxController debugController =
            new CommandXboxController(Constants.Controller.DEBUG_CONTROLLER_CHANNEL);

    // A chooser for autonomous commands
    private final SendableChooser<Command> autoChooser;

    // Creating 2d field in Sim/ShuffleBoard
    // Trying to get feedback from auto
    private final List<Pose2d> currentPath = new ArrayList<Pose2d>();

    // The constraints for this path.
    public static final PathConstraints SPEED_CONSTRAINTS = new PathConstraints(2, 1.5, 1.5 * Math.PI, 1 * Math.PI);

    public RobotContainer() {
        this.driveTrain.setName("DriveTrain");
        this.fuelSubsystem.setName("FuelSubsystem");
        this.visionSubsystem.setName("VisionSubsystem");

        this.configureShuffleboard();
        this.configureBindings();
        this.registerCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.
        // If AutoBuilder is not configured (no RobotConfig), create a basic chooser
        this.autoChooser = buildAutoChooserSafe();
    }

    /**
     * Safely builds an auto chooser, falling back to a basic chooser if AutoBuilder is not configured.
     */
    private SendableChooser<Command> buildAutoChooserSafe() {
        try {
            SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
            SmartDashboard.putData("Auto Chooser", chooser);
            // Creates a field to be put to the shuffleboard
            SmartDashboard.putData("AUTOPOSITION", (s) -> AutoBuilder.getCurrentPose());
            return chooser;
        } catch (RuntimeException e) {
            System.out.println("WARNING: AutoBuilder not configured. Creating basic auto chooser with none() command.");
            SendableChooser<Command> chooser = new SendableChooser<>();
            chooser.setDefaultOption("None", Commands.none());
            SmartDashboard.putData("Auto Chooser", chooser);
            return chooser;
        }
    }

    public Drivetrain getDriveTrain() {
        return driveTrain;
    }

    public FuelSubsystem getFuelSubsystem() {
        return fuelSubsystem;
    }

    public LedStrand getLedStrand() {
        return ledStrand;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public void periodic() {
        // us trying to set pose for field2d
    }

    private void registerCommands() {
        // Register commands for autonomous routines
        NamedCommands.registerCommand("ResetOdom", this.driveTrain.getResetOdometryCommand());
        NamedCommands.registerCommand("ToggleFieldRelative", this.driveTrain.getToggleFieldRelativeCommand());
        NamedCommands.registerCommand("StopDrive", this.driveTrain.getStopModulesCommand());

        // Fuel subsystem commands
        NamedCommands.registerCommand("FuelIntake", this.fuelSubsystem.getIntakeCommand());
        NamedCommands.registerCommand("FuelLaunch", this.fuelSubsystem.getLaunchCommand());
        NamedCommands.registerCommand("FuelSpinUp", this.fuelSubsystem.getSpinUpCommand());
        NamedCommands.registerCommand("FuelStop", this.fuelSubsystem.getStopCommand());
    }

    /**
     * Creates Command Bindings. Read description down below:
     *
     * <p>
     * Use this method to define your trigger->comand mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
     * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
     * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
     */
    private void configureBindings() {
        /**
         * Swerve Drive Controller Command
         *
         * <p>
         * Controls:
         * - Left Stick: Steering
         * - Right Stick: Rotate the robot
         * - Right Trigger: provide gas
         * - Left Trigger: reduce maximum driving speed by 50%
         */
        this.driveTrain.setDefaultCommand(new SwerveDriveCommand(
                ControllerDelegate.builder()
                        .leftXSupplier(this.driverController::getLeftX)
                        .leftYSupplier(this.driverController::getLeftY)
                        .rightXSupplier(this.driverController::getRightX)
                        .rightYSupplier(this.driverController::getRightY)
                        .accelerationSupplier(this.driverController::getRightTriggerAxis)
                        .elevatorDecelerateRatioSupplier(() -> 1.0) // No elevator, always full speed
                        .runHalfSpeedConditionSupplier(() -> driverController.getLeftTriggerAxis() >= 0.5)
                        .driver(ControllerDelegate.Driver.ASA)
                        .build(),
                driveTrain));

        // Driver Controller commands
        this.driverController.rightStick().onTrue(this.driveTrain.toggleWheelLockCommand()); // lock wheels
        this.driverController.b().onTrue(this.driveTrain.resetNavXSensorModule());

        // Manipulator Controller - Fuel Subsystem commands
        this.manipController.leftBumper().whileTrue(this.fuelSubsystem.getIntakeCommand());
        this.manipController
                .rightBumper()
                .whileTrue(this.fuelSubsystem
                        .getSpinUpCommand()
                        .withTimeout(1.0)
                        .andThen(this.fuelSubsystem.getLaunchCommand()));
        this.manipController.x().whileTrue(this.fuelSubsystem.getEjectCommand());
    }

    private void configureShuffleboard() {
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());

        // Add subsystems
        SmartDashboard.putData(this.driveTrain);
        SmartDashboard.putData(this.driveTrain.getName() + "/Reset Pose 2D", this.driveTrain.getResetOdometryCommand());
        SmartDashboard.putData(this.fuelSubsystem);
        SmartDashboard.putData(this.visionSubsystem);
    }

    // loads New Auto auto file
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
