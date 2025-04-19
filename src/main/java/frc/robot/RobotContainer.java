package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaePID;
import frc.robot.commands.AprilAlignCommand;
import frc.robot.commands.ElevatorPID;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.*;

import java.util.ArrayList;
import java.util.List;


public class RobotContainer {

    private final DriveTrain driveTrain = new DriveTrain(Constants.defaultRobotVersion);

    private final Limelight limelightFrl = new Limelight("limelight-frl");

    private final Limelight limelightFrr = new Limelight("limelight-frr");

    private final Limelight limelightBack = new Limelight("limelight-back");

    private final LedStrand ledStrand = new LedStrand();

    private final CoralMechanism coralMechanism = new CoralMechanism();

    private final ClimbMechanism climbMechanism = new ClimbMechanism();

    private final AlgaeMechanism algaeMechanism = new AlgaeMechanism();

    private final AlgaeIntake algaeIntake = new AlgaeIntake();

    private final ElevatorMechanism elevatorMechanism = new ElevatorMechanism();

    private final ElevatorPID elevatorPIDDown = new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.Down);
    private final ElevatorPID elevatorPIDL2 = new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L2);
    private final ElevatorPID elevatorPIDL3 = new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L3);
    private final ElevatorPID elevatorPIDL4 = new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L4);
    private final ElevatorPID elevatorPIDAlgae3 = new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.AlgaeL3);

    private final Servo servo = new Servo();

    private final AprilAlignCommand limelightCodeFrontLeft = new AprilAlignCommand(this.limelightFrl::getCurrentAprilTag, this.limelightFrl::getAprilRotation2d, this.driveTrain, new Transform2d(.18, 0.00, new Rotation2d(.15)), false, true, this.ledStrand);
    private final AprilAlignCommand limelightCodeFrontRight = new AprilAlignCommand(this.limelightFrr::getCurrentAprilTag, this.limelightFrr::getAprilRotation2d, this.driveTrain, new Transform2d(.07, 0.00, new Rotation2d(0.17)), false, false, this.ledStrand);
    private final AprilAlignCommand limelightCodeBack = new AprilAlignCommand(this.limelightBack::getCurrentAprilTag, this.limelightBack::getAprilRotation2d, this.driveTrain, new Transform2d(.65, 0.00, new Rotation2d()), true, false, this.ledStrand);

    private final AlgaePID algaePIDDown = new AlgaePID(algaeMechanism, AlgaeMechanism.PosDown);
    private final AlgaePID algaePIDMiddle = new AlgaePID(algaeMechanism, AlgaeMechanism.PosMiddle);
    private final AlgaePID algaePIDUp = new AlgaePID(algaeMechanism, AlgaeMechanism.PosUp);
    private final AlgaePID algaePIDGround = new AlgaePID(algaeMechanism, AlgaeMechanism.PosGround);

    // Compose the commands correctly, ensuring that each use is a new composition
    private final Command runCoralForward = this.coralMechanism.CoralForwardCmd().onlyWhile(() -> !this.coralMechanism.IsCoralLoaded()).withName("RunCoral");
    private final Command algaeDownAndRunA3 = Commands.race(new AlgaePID(algaeMechanism, AlgaeMechanism.PosDown), algaeIntake.IntakeForwardOnceCmd(), new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.AlgaeL3));
    private final Command algaeDownAndRunA4 = Commands.race(new AlgaePID(algaeMechanism, AlgaeMechanism.PosDown), algaeIntake.IntakeForwardOnceCmd(), new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.AlgaeL4));
    private final Command algaeUpAndStop = Commands.race(new AlgaePID(algaeMechanism, AlgaeMechanism.PosUp), algaeIntake.IntakeStopCmd());
    private final Command algaeMiddleAndStop = Commands.race(new AlgaePID(algaeMechanism, AlgaeMechanism.PosMiddle), algaeIntake.IntakeStopCmd());
    private final Command algaeGroundCommand = Commands.race(new AlgaePID(algaeMechanism, AlgaeMechanism.PosGround), algaeIntake.IntakeBackwardOnceCmd(), new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L2));

    /**
     * Creates buttons and controller for: - the driver controller (port 0) - the manipulator
     * controller (port 1) - the debug controller (port 2)
     */
    private final CommandXboxController driverController = new CommandXboxController(Constants.Controller.DriverControllerChannel);
    private final CommandXboxController manipController = new CommandXboxController(Constants.Controller.ManipControllerChannel);
    private final CommandXboxController debugController = new CommandXboxController(Constants.Controller.DebugControllerChannel);

    // A chooser for autonomous commands
    private final SendableChooser<Command> autoChooser;

    // Creating 2d field in Sim/ShuffleBoard
    // Trying to get feedback from auto
    private final List<Pose2d> currentPath = new ArrayList<Pose2d>();
    private final Command resetPoseAuto = Commands.runOnce(() -> driveTrain.resetPose(this.currentPath.get(0)), this.driveTrain);

    public static final PathConstraints SPEED_CONSTRAINTS = new PathConstraints(2, 1.5, 1.5 * Math.PI, 1 * Math.PI); // The constraints for this path.

    public RobotContainer() {
        this.elevatorMechanism.setName("ElevatorMechanism");
        this.driveTrain.setName("DriveTrain");
        this.coralMechanism.setName("CoralMechanism");
        this.algaeIntake.setName("AlgaeIntake");
        this.algaeMechanism.setName("AlgaeMechanism");

        this.configureShuffleboard();
        this.configureBindings();
        this.registerCommands();

        // Build an auto chooser. This will use Commands.none() as the default option.

        autoChooser = AutoBuilder.buildAutoChooser();


        // Another option that allows you to specify the default auto by its name:
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        // Creates a field to be put to the shuffleboard
        SmartDashboard.putData("AUTOPOSITION", (s) -> AutoBuilder.getCurrentPose());

    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }

    public Limelight getLimelightFrl() {
        return limelightFrl;
    }

    public Limelight getLimelightFrr() {
        return limelightFrr;
    }

    public Limelight getLimelightBack() {
        return limelightBack;
    }

    public LedStrand getLedStrand() {
        return ledStrand;
    }

    public void periodic() {
        // us trying to set pose for field2d
    }

    private void registerCommands() {
        NamedCommands.registerCommand("Limelight", this.limelightCodeFrontLeft);
        NamedCommands.registerCommand("LimelightSetFirstLeftPriority", this.limelightFrl.PriorityIDcmd(22, 9));
        NamedCommands.registerCommand("LimelightSetSecondLeftPriority", this.limelightFrl.PriorityIDcmd(17, 8));
        NamedCommands.registerCommand("LimelightSetSecondLeftPriority2", this.limelightFrl.PriorityIDcmd(18, 8));
        NamedCommands.registerCommand("LimelightSetFirstRightPriority", this.limelightFrl.PriorityIDcmd(20, 11));
        NamedCommands.registerCommand("LimelightSetSecondRightPriority", this.limelightFrl.PriorityIDcmd(19, 6));
        NamedCommands.registerCommand("LimelightBack", this.limelightCodeBack);
        NamedCommands.registerCommand("RobotOrientedLimelight", this.limelightFrl.setLimelightUsageField());
        NamedCommands.registerCommand("SETPOSEfrl", resetPoseAuto);
        NamedCommands.registerCommand("PathRESETODMLeft", AutoBuilder.resetOdom(new Pose2d(5.002, 2.806, new Rotation2d(90))));
        NamedCommands.registerCommand("PathRESETODMRight", AutoBuilder.resetOdom(new Pose2d(5.021, 5.253, new Rotation2d(180))));
        NamedCommands.registerCommand("PathRESETODMMiddle", AutoBuilder.resetOdom(new Pose2d(5.802, 3.959, new Rotation2d(180))));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L2));
        NamedCommands.registerCommand("ElevatorA3", this.algaeDownAndRunA3);
        NamedCommands.registerCommand("ElevatorA4", this.algaeDownAndRunA4);
        NamedCommands.registerCommand("IntakeCoral", this.coralMechanism.CoralIntakeAutoCmd());
        NamedCommands.registerCommand("ResetOdom", this.driveTrain.resetPose2d());
        NamedCommands.registerCommand("ElevatorL4", new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.L4).onlyWhile(() -> !this.elevatorMechanism.ElevatorAtPos()));
        NamedCommands.registerCommand("ElevatorBottom", new ElevatorPID(this.elevatorMechanism, ElevatorMechanism.Down));
        NamedCommands.registerCommand("ElevatorUp", this.elevatorMechanism.ElevatorUpCmd());
        NamedCommands.registerCommand("ShootCoral", this.coralMechanism.CoralForwardCmd().withTimeout(.5));
        NamedCommands.registerCommand("ToggleFieldRelativel", this.driveTrain.toggleFieldRelativeEnable());
        NamedCommands.registerCommand("WaitUntilElevatorTop", new WaitUntilCommand(this.elevatorMechanism::ElevatorAtPos));
        NamedCommands.registerCommand("StopDrive", this.driveTrain.Break());
    }

    /**
     * Creates Command Bindings. Read description down below:
     *
     * <p>Use this method to define your trigger->comand mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        /**
         * Swerve Drive Controller Command
         *
         * <p>Controls: - Left Stick: Steering - Right Stick: Rotate the robot - Right Trigger: provide
         * gas - Left Trigger: reduce maximum driving speed by 50% RECOMMENDED TO USE
         */
        this.driveTrain.setDefaultCommand(new SwerveDriveCommand(this.driverController::getLeftY, this.driverController::getLeftX, this.driverController::getRightX, this.driverController::getRightTriggerAxis, this.elevatorMechanism::getElevatorDecelerateRatio, driveTrain, () -> driverController.getLeftTriggerAxis() >= 0.5, "Asa"));


        // Driver Controller commands
        // - DriveTrain commands (outside of actual driving)
        this.driverController.a().whileTrue(this.limelightCodeBack);
        this.driverController.rightStick().onTrue(this.driveTrain.WheelLockCommand()); // lock wheels
        this.driverController.x().whileTrue(this.limelightCodeFrontLeft);
        this.driverController.y().whileTrue(this.limelightCodeFrontRight);
        this.driverController.b().onTrue(this.driveTrain.ZeroGyro());

        //Elevator Commands
        this.manipController.a().onTrue(this.elevatorPIDDown);
        this.manipController.b().onTrue(this.elevatorPIDL2);
        this.manipController.y().onTrue(this.elevatorPIDL3);
        this.manipController.x().onTrue(this.elevatorPIDL4);
        this.manipController.rightStick().onTrue(this.elevatorMechanism.ResetPositionCMD());
        //Algae Commands
        this.manipController.leftBumper().onTrue(this.algaeDownAndRunA3);
        this.manipController.rightBumper().onTrue(this.algaeDownAndRunA4);
        this.manipController.back().onTrue(this.algaeUpAndStop);
        this.manipController.start().whileTrue(this.algaeIntake.IntakeBackwardCmd());
        this.manipController.povLeft().onTrue(this.algaeGroundCommand);
        this.manipController.povRight().whileTrue(this.algaeIntake.IntakeForwardCmd());
        this.manipController.leftStick().onTrue(this.algaeMiddleAndStop);

        //Coral Commands
        this.manipController.leftTrigger(.5).whileTrue(this.runCoralForward);
        this.manipController.rightTrigger(.5).whileTrue(this.coralMechanism.CoralForwardCmd());
        this.manipController.povUp().whileTrue(this.coralMechanism.CoralTrophCmd());
        this.manipController.povDown().whileTrue(this.climbMechanism.WindDownCmd());
        this.servo.setDefaultCommand(this.servo.ServoForwardCommand());

        this.debugController.rightBumper().whileTrue(this.elevatorMechanism.ElevatorDownLimitCmd());
        this.debugController.leftBumper().whileTrue(this.elevatorMechanism.ElevatorUpLimitCmd());
        this.debugController.povUp().onTrue(this.algaeMechanism.resetAlgae());
        this.debugController.a().onTrue(this.algaePIDUp);
        this.debugController.b().onTrue(this.algaePIDMiddle);
        this.debugController.x().onTrue(this.algaePIDDown);
        this.debugController.y().onTrue(this.algaePIDGround);

        this.debugController.rightStick().onTrue(this.algaeDownAndRunA4);
        this.debugController.povDown().onTrue(this.elevatorPIDAlgae3);
        this.debugController.leftStick().whileTrue(this.algaeIntake.IntakeBackwardCmd());
        this.debugController.start().onTrue(this.algaeMiddleAndStop);
        this.debugController.leftTrigger(.5).onTrue(this.algaeGroundCommand);
    }

    private void configureShuffleboard() {
        SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        SmartDashboard.putData(this.elevatorMechanism);


        // Add subsystems
        SmartDashboard.putData(this.driveTrain);
        SmartDashboard.putData(this.driveTrain.getName() + "/Reset Pose 2D", this.driveTrain.resetPose2d());
        SmartDashboard.putData(this.coralMechanism);
        ;
        SmartDashboard.putData(this.algaeMechanism);
        SmartDashboard.putData(this.algaeIntake);

    }

    // loads New Auto auto file
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
