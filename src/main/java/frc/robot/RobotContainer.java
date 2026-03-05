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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.ClimbTestCommand;
import frc.robot.commands.auto.VisionAlignmentTestCommand;
import frc.robot.commands.swervedrive.ControllerDelegate;
import frc.robot.commands.swervedrive.SwerveDriveCommand;
import frc.robot.subsystems.LedStrand;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystemContext;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.DrivetrainContext;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystemContext;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystemContext;
import frc.robot.subsystems.relay.RelaySubsystem;
import frc.robot.subsystems.relay.RelaySubsystemContext;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystemContext;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.turret.TurretSubsystemContext;
import frc.robot.subsystems.turrettracker.TurretTracker;
import frc.robot.subsystems.turrettracker.TurretTrackerContext;
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

    private final IntakeSubsystem intakeSubsystem;

    private final RelaySubsystem relaySubsystem;

    private final IndexerSubsystem indexerSubsystem;

    private final ShooterSubsystem shooterSubsystem;

    private final TurretSubsystem turretSubsystem;

    private final VisionSubsystem visionSubsystem;

    private final TurretTracker turretTracker;

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
        // Conditionally construct subsystems based on feature flags
        this.ledStrand = Constants.FeatureFlags.ENABLE_LED_STRAND ? new LedStrand() : null;

        this.climbSubsystem = Constants.FeatureFlags.ENABLE_CLIMB
                ? new ClimbSubsystem(ClimbSubsystemContext.defaults(), this.driveTrain)
                : null;

        this.intakeSubsystem =
                Constants.FeatureFlags.ENABLE_INTAKE ? new IntakeSubsystem(IntakeSubsystemContext.defaults()) : null;

        this.relaySubsystem =
                Constants.FeatureFlags.ENABLE_RELAY ? new RelaySubsystem(RelaySubsystemContext.defaults()) : null;

        this.indexerSubsystem =
                Constants.FeatureFlags.ENABLE_INDEXER ? new IndexerSubsystem(IndexerSubsystemContext.defaults()) : null;

        this.shooterSubsystem =
                Constants.FeatureFlags.ENABLE_SHOOTER ? new ShooterSubsystem(ShooterSubsystemContext.defaults()) : null;

        this.turretSubsystem =
                Constants.FeatureFlags.ENABLE_TURRET ? new TurretSubsystem(TurretSubsystemContext.defaults()) : null;

        this.visionSubsystem = Constants.FeatureFlags.ENABLE_VISION
                ? new VisionSubsystem(
                        VisionSubsystemContext.builder()
                                .enablePhotonCameraSimStreams(true)
                                .build(),
                        driveTrain,
                        driveTrain::addVisionMeasurement)
                : null;

        this.turretTracker = Constants.FeatureFlags.ENABLE_TURRET_TRACKER
                ? new TurretTracker(TurretTrackerContext.defaults(), driveTrain)
                : null;

        this.driveTrain.setName("DriveTrain");
        if (this.climbSubsystem != null) this.climbSubsystem.setName("ClimbSubsystem");
        if (this.intakeSubsystem != null) this.intakeSubsystem.setName("IntakeSubsystem");
        if (this.relaySubsystem != null) this.relaySubsystem.setName("RelaySubsystem");
        if (this.indexerSubsystem != null) this.indexerSubsystem.setName("IndexerSubsystem");
        if (this.shooterSubsystem != null) this.shooterSubsystem.setName("ShooterSubsystem");
        if (this.turretSubsystem != null) this.turretSubsystem.setName("TurretSubsystem");
        if (this.visionSubsystem != null) this.visionSubsystem.setName("VisionSubsystem");
        if (this.turretTracker != null) this.turretTracker.setName("TurretTracker");

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
            Telemetry.putData("Auto Chooser", chooser);
            // Creates a field to be put to the shuffleboard
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

    public IntakeSubsystem getIntakeSubsystem() {
        return intakeSubsystem;
    }

    public LedStrand getLedStrand() {
        return ledStrand;
    }

    public VisionSubsystem getVisionSubsystem() {
        return visionSubsystem;
    }

    public TurretTracker getTurretTracker() {
        return turretTracker;
    }

    public void periodic() {
        // us trying to set pose for field2d
    }

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
        // Register commands for autonomous routines
        NamedCommands.registerCommand("ResetOdom", this.driveTrain.getResetOdometryCommand());
        NamedCommands.registerCommand("ToggleFieldRelative", this.driveTrain.getToggleFieldRelativeCommand());
        NamedCommands.registerCommand("StopDrive", this.driveTrain.getStopModulesCommand());

        // Climb subsystem commands (only if climb is enabled)
        if (this.climbSubsystem != null) {
            // Auto command: extends to bar 1, then partial retract to lift off ground
            NamedCommands.registerCommand("ClimbAuto", this.climbSubsystem.getRetractToAutoHeightCommand());
            // Utility commands usable in autos or named sequences
            NamedCommands.registerCommand("ClimbHome", this.climbSubsystem.getHomingCommand());
            NamedCommands.registerCommand("ClimbExtend", this.climbSubsystem.getExtendToBarCommand());
            NamedCommands.registerCommand("ClimbNextBar", this.climbSubsystem.getClimbNextBarCommand());
            NamedCommands.registerCommand("ClimbStop", this.climbSubsystem.getStopCommand());
        }

        // Intake subsystem commands (only if intake is enabled)
        if (this.intakeSubsystem != null) {
            NamedCommands.registerCommand("IntakeHome", this.intakeSubsystem.getHomingCommand());
            NamedCommands.registerCommand("IntakeRaise", this.intakeSubsystem.getRaiseCommand());
            NamedCommands.registerCommand("IntakeLower", this.intakeSubsystem.getLowerCommand());
            NamedCommands.registerCommand("IntakeIntake", this.intakeSubsystem.getIntakeCommand());
            NamedCommands.registerCommand("IntakeStop", this.intakeSubsystem.getStopCommand());
        }
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
         */
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

        // Driver Controller commands
        this.driverController.rightStick().onTrue(this.driveTrain.toggleWheelLockCommand()); // lock wheels
        this.driverController.b().onTrue(this.driveTrain.resetNavXSensorModule());

        // Manipulator Controller - Climb Subsystem commands (only if climb is enabled)
        // TODO: Confirm all button assignments with drive team before first climb test.
        //
        // A button    → climb next bar (retract to engage hooks; auto-stops)
        // B button    → extend telescope up to next bar (position-based, auto-stops)
        // Back button → manual re-home (use if climb drifted or auto-home didn't complete cleanly)
        //
        // TODO: Manual retract/extend overrides need a new home — LB and RB are now used by
        //       intake (LB) and the future reverse-all command (RB). Candidates: chord (Back+A/B),
        //       debug controller, or stick-click buttons.
        if (this.climbSubsystem != null) {
            this.manipController.a().onTrue(this.climbSubsystem.getClimbNextBarCommand());
            this.manipController.b().onTrue(this.climbSubsystem.getExtendToBarCommand());
            this.manipController.back().onTrue(this.climbSubsystem.getHomingCommand());
        }

        // Manipulator Controller - Intake commands (only if intake is enabled)
        // TODO: Confirm all button assignments with drive team before first intake test.
        //
        // Manip LT (held)    → intake rollers spin in to collect balls
        // Manip LB (held)    → intake rollers reverse to eject
        // Manip Y button     → raise intake to stowed position (for climb)
        // Manip X button     → lower intake to match position
        // Manip Start button → re-home the intake lift (raises to retracted hardstop, zeros encoders)
        if (this.intakeSubsystem != null) {
            this.manipController.leftTrigger().whileTrue(this.intakeSubsystem.getIntakeCommand());
            this.manipController.leftBumper().whileTrue(this.intakeSubsystem.getReverseCommand());
            this.manipController.y().onTrue(this.intakeSubsystem.getRaiseCommand());
            this.manipController.x().onTrue(this.intakeSubsystem.getLowerCommand());
            this.manipController.start().onTrue(this.intakeSubsystem.getHomingCommand());
        }

        // Manipulator Controller - Relay + Indexer + Shooter commands (only if all three are enabled)
        //
        // Manip RT (held) → advance relay + indexer + shooter in unison to fire balls
        // Manip RB (held) → reverse relay + indexer + shooter in unison to clear jams
        //
        // TODO: Once the physics-based inverse solver (SHOOTER.md) is integrated, the shooter
        //       command will accept a distance-to-target supplier instead of running open-loop.
        if (this.relaySubsystem != null && this.indexerSubsystem != null && this.shooterSubsystem != null) {
            this.manipController
                    .rightTrigger()
                    .whileTrue(Commands.parallel(
                            this.relaySubsystem.getRunCommand(),
                            this.indexerSubsystem.getIndexCommand(),
                            this.shooterSubsystem.getShootCommand()));
            this.manipController
                    .rightBumper()
                    .whileTrue(Commands.parallel(
                            this.relaySubsystem.getReverseCommand(),
                            this.indexerSubsystem.getReverseCommand(),
                            this.shooterSubsystem.getReverseCommand()));
        }

        // Turret default command — track TurretTracker angle when both are enabled.
        // When only the turret motor is enabled (tracker disabled), turret stays IDLE.
        if (this.turretSubsystem != null && this.turretTracker != null) {
            this.turretSubsystem.setDefaultCommand(
                    this.turretSubsystem.getTrackCommand(this.turretTracker::getTurretAngleDegrees));
        }

        // Debug Controller - Turret manual jog commands (only if turret is enabled)
        // Used during bring-up to verify motor direction and encoder sign convention.
        //
        // Debug D-pad left  (held) → jog turret left  (CCW; should produce positive encoder counts)
        // Debug D-pad right (held) → jog turret right (CW;  should produce negative encoder counts)
        //
        // TODO: Remove or gate behind a sim/lab mode once closed-loop tracking is verified.
        if (this.turretSubsystem != null) {
            this.debugController.povLeft().whileTrue(this.turretSubsystem.getJogLeftCommand());
            this.debugController.povRight().whileTrue(this.turretSubsystem.getJogRightCommand());
            this.debugController.back().onTrue(this.turretSubsystem.getResetTurretRotationCommand());
        }

        // Debug Controller - Relay selective run (only if relay is enabled)
        // Allows the relay to be exercised in isolation, independent of the indexer and shooter.
        // Use this during bring-up to verify roller direction and speed before enabling the full
        // manip RT/RB fuel-delivery chain.
        //
        // Debug RT (held) → run relay forward  (conveys balls toward indexer)
        // Debug RB (held) → run relay reverse  (clears jams)
        if (this.relaySubsystem != null) {
            this.debugController.rightTrigger().whileTrue(this.relaySubsystem.getRunCommand());
            this.debugController.rightBumper().whileTrue(this.relaySubsystem.getReverseCommand());
        }

        // Debug Controller - Shooter flywheel speed tuning
        // Hold LT to spin both flywheels at the current setpoints, then tap face buttons to adjust.
        // The adjust commands do not require the shooter subsystem, so they can be tapped
        // concurrently while LT is held — runForward() picks up the new value on the next loop.
        //
        // Debug LT (held) → run shooter at current setpoints  (listen / measure)
        // Debug Y  (tap)  → front flywheel speed +2%
        // Debug A  (tap)  → front flywheel speed -2%
        // Debug B  (tap)  → rear  flywheel speed +2%
        // Debug X  (tap)  → rear  flywheel speed -2%
        //
        // Watch Shooter/Front/SpeedSetpoint and Shooter/Rear/SpeedSetpoint in telemetry
        // to confirm the current values before writing them back to Constants.
        if (this.shooterSubsystem != null) {
            this.debugController.leftTrigger().whileTrue(this.shooterSubsystem.getShootCommand());
            this.debugController.y().onTrue(this.shooterSubsystem.getIncreaseFrontSpeedCommand());
            this.debugController.a().onTrue(this.shooterSubsystem.getDecreaseFrontSpeedCommand());
            this.debugController.b().onTrue(this.shooterSubsystem.getIncreaseRearSpeedCommand());
            this.debugController.x().onTrue(this.shooterSubsystem.getDecreaseRearSpeedCommand());
        }
    }

    private void configureShuffleboard() {
        Telemetry.putData("Command Scheduler", CommandScheduler.getInstance());

        // Add subsystems
        Telemetry.putData(this.driveTrain);
        Telemetry.putData(this.driveTrain.getName() + "/Reset Pose 2D", this.driveTrain.getResetOdometryCommand());
        if (this.climbSubsystem != null) Telemetry.putData(this.climbSubsystem);
        if (this.intakeSubsystem != null) Telemetry.putData(this.intakeSubsystem);
        if (this.relaySubsystem != null) Telemetry.putData(this.relaySubsystem);
        if (this.indexerSubsystem != null) Telemetry.putData(this.indexerSubsystem);
        if (this.shooterSubsystem != null) Telemetry.putData(this.shooterSubsystem);
        if (this.turretSubsystem != null) Telemetry.putData(this.turretSubsystem);
        if (this.visionSubsystem != null) Telemetry.putData(this.visionSubsystem);
        if (this.turretTracker != null) Telemetry.putData(this.turretTracker);

        // Vision alignment test command (for simulation testing)
        if (this.visionSubsystem != null) {
            VisionAlignmentTestCommand.create(this.driveTrain)
                    .ifPresent(cmd -> Telemetry.putData("Vision/AlignmentTest", cmd));
        }

        // Climb test command (for simulation testing)
        if (this.climbSubsystem != null) {
            ClimbTestCommand.create(this.climbSubsystem).ifPresent(cmd -> Telemetry.putData("Climb/ClimbTest", cmd));
        }
    }

    // loads New Auto auto file
    public Command getAutonomousCommand() {
        return this.autoChooser.getSelected();
    }
}
