package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.ClimbTestCommand;
import frc.robot.commands.auto.VisionAlignmentTestCommand;
import frc.robot.commands.swervedrive.ControllerDelegate;
import frc.robot.commands.swervedrive.SwerveDriveCommand;
import frc.robot.subsystems.LedStrand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystemContext;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainContext;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemContext;
import frc.robot.support.Telemetry;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {

    private final LedStrand ledStrand;

    private final DrivetrainContext drivetrainContext = DrivetrainContext.defaults();

    private final Drivetrain driveTrain = new Drivetrain(drivetrainContext);

    private final ClimbSubsystem climbSubsystem;

    private final VisionSubsystem visionSubsystem;

    private final Command resetPoseAuto =
            Commands.runOnce(() -> this.driveTrain.resetOdometry(this.currentPath.get(0)), this.driveTrain);

    private final CommandXboxController driverController =
            new CommandXboxController(Constants.Controller.DRIVER_CONTROLLER_CHANNEL);

    private final CommandXboxController manipController =
            new CommandXboxController(Constants.Controller.MANIPULATION_CONTROLLER_CHANNEL);

    private final CommandXboxController debugController =
            new CommandXboxController(Constants.Controller.DEBUG_CONTROLLER_CHANNEL);

    private final SendableChooser<Command> autoChooser;

    private final List<Pose2d> currentPath = new ArrayList<Pose2d>();

    public static final PathConstraints SPEED_CONSTRAINTS = new PathConstraints(2, 1.5, 1.5 * Math.PI, 1 * Math.PI);

    public RobotContainer() {
        this.ledStrand = Constants.FeatureFlags.ENABLE_LED_STRAND ? new LedStrand() : null;

        this.climbSubsystem = Constants.FeatureFlags.ENABLE_CLIMB
                ? new ClimbSubsystem(ClimbSubsystemContext.defaults(), this.driveTrain)
                : null;

        this.visionSubsystem = Constants.FeatureFlags.ENABLE_VISION
                ? new VisionSubsystem(
                        VisionSubsystemContext.builder()
                                .enablePhotonCameraSimStreams(true)
                                .build(),
                        driveTrain,
                        driveTrain::addVisionMeasurement)
                : null;

        this.driveTrain.setName("DriveTrain");
        if (this.climbSubsystem != null) this.climbSubsystem.setName("ClimbSubsystem");
        if (this.visionSubsystem != null) this.visionSubsystem.setName("VisionSubsystem");

        this.configureShuffleboard();
        this.configureBindings();
        this.registerCommands();

        this.autoChooser = buildAutoChooserSafe();
    }

    private SendableChooser<Command> buildAutoChooserSafe() {
        try {
            SendableChooser<Command> chooser = AutoBuilder.buildAutoChooser();
            Telemetry.putData("Auto Chooser", chooser);
            Telemetry.putData("AUTOPOSITION", (s) -> AutoBuilder.getCurrentPose());
            return chooser;
        } catch (RuntimeException e) {
            System.out.println("WARNING: AutoBuilder not configured. Creating basic auto chooser with none() command.");
            SendableChooser<Command> chooser = new SendableChooser<>();
            chooser.setDefaultOption("None", Commands.none());
            Telemetry.putData("Auto Chooser", chooser);
            return chooser;
        }
    }

    public Drivetrain getDriveTrain() {
        return driveTrain;
    }

    public ClimbSubsystem getClimbSubsystem() {
        return climbSubsystem;
    }

    public LedStrand getLedStrand() {
        return ledStrand;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public void periodic() {}

    /**
     * Called by {@link Robot#teleopInit()} to schedule any subsystem init routines for teleop.
     *
     * <p>For the climb subsystem: first lowers the robot to the ground (in case it was lifted
     * during auto), then re-homes the encoder so it is valid for the full teleop climb cycle.
     * If the robot was never lifted, the lower command exits immediately and homing proceeds.
     *
     * <p><strong>The lower step is required, not just convenient</strong> — there is no internal
     * hardstop between the telescope stages. Homing relies on ground contact to produce the
     * current spike that zeroes the encoder.
     */
    public void scheduleTeleopInit() {
        if (this.climbSubsystem != null) {
            this.climbSubsystem
                    .getLowerToGroundCommand()
                    .andThen(this.climbSubsystem.getHomingCommand())
                    .schedule();
        }
    }

    private void registerCommands() {
        NamedCommands.registerCommand("ResetOdom", this.driveTrain.getResetOdometryCommand());
        NamedCommands.registerCommand("ToggleFieldRelative", this.driveTrain.getToggleFieldRelativeCommand());
        NamedCommands.registerCommand("StopDrive", this.driveTrain.getStopModulesCommand());

        if (this.climbSubsystem != null) {
            NamedCommands.registerCommand("ClimbAuto", this.climbSubsystem.getRetractToAutoHeightCommand());
            NamedCommands.registerCommand("ClimbHome", this.climbSubsystem.getHomingCommand());
            NamedCommands.registerCommand("ClimbExtend", this.climbSubsystem.getExtendToBarCommand());
            NamedCommands.registerCommand("ClimbNextBar", this.climbSubsystem.getClimbNextBarCommand());
            NamedCommands.registerCommand("ClimbStop", this.climbSubsystem.getStopCommand());
        }
    }

    private void configureBindings() {
        this.driveTrain.setDefaultCommand(new SwerveDriveCommand(
                ControllerDelegate.builder()
                        .leftXSupplier(this.driverController::getLeftX)
                        .leftYSupplier(this.driverController::getLeftY)
                        .rightXSupplier(this.driverController::getRightX)
                        .rightYSupplier(this.driverController::getRightY)
                        .accelerationSupplier(this.driverController::getRightTriggerAxis)
                        .elevatorDecelerateRatioSupplier(() -> 1.0) // No elevator, always full speed
                        .driver(ControllerDelegate.Driver.ASA)
                        .build(),
                driveTrain));

        this.driverController.rightStick().onTrue(this.driveTrain.toggleWheelLockCommand());
        this.driverController.b().onTrue(this.driveTrain.resetNavXSensorModule());

        // TODO: Confirm climb button assignments with drive team
        // TODO: Manual retract/extend overrides need a new home (LB/RB taken by intake)
        if (this.climbSubsystem != null) {
            this.manipController.a().onTrue(this.climbSubsystem.getClimbNextBarCommand());
            this.manipController.b().onTrue(this.climbSubsystem.getExtendToBarCommand());
            this.manipController.back().onTrue(this.climbSubsystem.getHomingCommand());
        }
    }

    private void configureShuffleboard() {
        Telemetry.putData("Command Scheduler", CommandScheduler.getInstance());
        Telemetry.putData(this.driveTrain);
        Telemetry.putData(this.driveTrain.getName() + "/Reset Pose 2D", this.driveTrain.getResetOdometryCommand());
        if (this.climbSubsystem != null) Telemetry.putData(this.climbSubsystem);
        if (this.visionSubsystem != null) Telemetry.putData(this.visionSubsystem);

        if (this.visionSubsystem != null) {
            VisionAlignmentTestCommand.create(this.driveTrain)
                    .ifPresent(cmd -> Telemetry.putData("Vision/AlignmentTest", cmd));
        }

        if (this.climbSubsystem != null) {
            ClimbTestCommand.create(this.climbSubsystem).ifPresent(cmd -> Telemetry.putData("Climb/ClimbTest", cmd));
        }
    }

    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
