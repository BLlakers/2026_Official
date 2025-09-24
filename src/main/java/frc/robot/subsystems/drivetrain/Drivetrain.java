// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static java.util.Objects.requireNonNull;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Limelights;
import frc.robot.sim.SwerveModuleSim;
import frc.robot.support.limelight.LimelightHelpers;
import java.util.Optional;

/**
 * Represents a swerve drive style drivetrain. In here, we initialize our swerve modules (example ->
 * {@link #frontLeftSwerveModule}), Get input from autonomous and initialize our odometry ->
 * {@link #swerveDriveOdometry}. Various other DriveTrain Related thing are initialized here too.
 */
public class Drivetrain extends SubsystemBase {

    private final DrivetrainContext context;

    private final Field2d field = new Field2d();

    private final SwerveDriveKinematics swerveDriveKinematics;

    // The gyro object. Gyro gives the robots rotation/ where the robot is pointed.
    private final AHRS navXSensorModule;

    public final SwerveModule frontLeftSwerveModule;

    public final SwerveModule frontRightSwerveModule;

    public final SwerveModule rearLeftSwerveModule;

    public final SwerveModule rearRightSwerveModule;

    private final SwerveModuleSim frontLeftSwerveModuleSim;

    private final SwerveModuleSim frontRightSwerveModuleSim;

    private final SwerveModuleSim rearLeftSwerveModuleSim;

    private final SwerveModuleSim rearRightSwerveModuleSim;

    private final SwerveDriveOdometry swerveDriveOdometry;

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

    // TODO: These publishers should
    private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher;

    private final StructArrayPublisher<SwerveModuleState> currentStatePublisher;

    private final StructPublisher<ChassisSpeeds> currentSpeedsPublisher;

    private final StructPublisher<Pose2d> currentPosePublisher;

    private final StructPublisher<Pose2d> currentPoseEstimatorPublisher;

    private final StructPublisher<Pose2d> goalPosePublisher;

    private final StructPublisher<Rotation2d> currentRotPublisher;

    private boolean wheelLock = false;

    private boolean fieldRelativeEnable = true;

    private Pose2d goalPose;

    private SwerveModuleState[] desiredStates;

    // Simulated gyro yaw (only used in simulation)
    private Rotation2d simYaw = new Rotation2d();

    private double lastSimTime = Timer.getFPGATimestamp();

    private SwerveModuleState[] CurrentStates; // TODO: Unused? Eliminate

    /**
     * Instantiates a new Drivetrain with default {@link DrivetrainContext}
     */
    public Drivetrain() {
        this(DrivetrainContext.defaults());
    }

    /**
     * Instantiates a new Drivetrain subsystem with the specified settings
     *
     * @param context
     *            The DrivetrainSettings to apply to this instance
     */
    public Drivetrain(final DrivetrainContext context) {
        requireNonNull(context, "DrivetrainContext cannot be null");

        this.context = context;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();

        this.desiredStatePublisher = nti.getStructArrayTopic("DesiredStates", SwerveModuleState.struct)
                .publish();

        this.currentStatePublisher = nti.getStructArrayTopic("CurrentStates", SwerveModuleState.struct)
                .publish();

        this.currentSpeedsPublisher =
                nti.getStructTopic("CurrentSpeed", ChassisSpeeds.struct).publish();

        this.currentPosePublisher =
                nti.getStructTopic("CurrentPose", Pose2d.struct).publish();

        this.currentPoseEstimatorPublisher =
                nti.getStructTopic("CurrentPoseEstimator", Pose2d.struct).publish();

        this.goalPosePublisher =
                nti.getStructTopic("GoalPoseEstimator", Pose2d.struct).publish();

        this.currentRotPublisher =
                nti.getStructTopic("CurrentRot", Rotation2d.struct).publish();

        this.navXSensorModule = new AHRS(AHRS.NavXComType.kMXP_SPI);

        this.context
                .getRobotConfig()
                .ifPresentOrElse(
                        robotConfig -> {
                            AutoBuilder.configure(
                                    this::getPose2d, // Robot pose supplier NEEDS TO BE POSE2D IF WE ARE USING OLD
                                    // LIMELIGHT WAY TODO
                                    this::resetOdometry, // Method to reset odometry (will be called if your auto
                                    // has a starting pose)
                                    // NEEDS TO BE RESETPOSE2D IF WE ARE USING OLD LIMELIGHT
                                    this::getChassisSpeeds,
                                    this::driveRobotRelative, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also,
                                    // optionally outputs
                                    // individual module feedforwards
                                    new PPHolonomicDriveController(
                                            new PIDConstants(this.context
                                                    .getLateralMovementPIDSettings()
                                                    .p()),
                                            new PIDConstants(this.context
                                                    .getRotationPIDSettings()
                                                    .p())),
                                    // new PPHolonomicDriveController(Translation,Rotation),
                                    // //PPHolonomicDriveController(Translation,
                                    // Rotation, .2),
                                    robotConfig, // The robot configuration
                                    () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        Optional<Alliance> alliance = DriverStation.getAlliance();
                                        return alliance.filter(value -> value == Alliance.Red)
                                                .isPresent();
                                    },
                                    this); // Reference to this subsystem to set requirements
                        },
                        () -> {
                            // NOTE: This should probably be fatal
                            throw new RuntimeException("Unable to obtain RobotConfig during Drivetrain creation."
                                    + "Path Planning will be fail!");
                        });

        this.swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.Drive.SMFrontLeftLocation,
                Constants.Drive.SMFrontRightLocation,
                Constants.Drive.SMBackLeftLocation,
                Constants.Drive.SMBackRightLocation);

        this.frontRightSwerveModule = new SwerveModule(this.context.getFrontRightSwerveModuleContext());
        this.frontLeftSwerveModule = new SwerveModule(this.context.getFrontLeftSwerveModuleContext());
        this.rearLeftSwerveModule = new SwerveModule(this.context.getRearLeftSwerveModuleContext());
        this.rearRightSwerveModule = new SwerveModule(this.context.getRearRightSwerveModuleContext());

        this.frontRightSwerveModuleSim = new SwerveModuleSim(context.getFrontRightSwerveModuleContext());
        this.frontLeftSwerveModuleSim = new SwerveModuleSim(context.getFrontLeftSwerveModuleContext());
        this.rearLeftSwerveModuleSim = new SwerveModuleSim(context.getRearLeftSwerveModuleContext());
        this.rearRightSwerveModuleSim = new SwerveModuleSim(context.getRearRightSwerveModuleContext());

        // initializes odometry
        this.swerveDriveOdometry = new SwerveDriveOdometry(
                this.swerveDriveKinematics, this.navXSensorModule.getRotation2d(), this.getSwerveModulePositions());

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                this.swerveDriveKinematics,
                this.navXSensorModule.getRotation2d(),
                this.getSwerveModulePositions(),
                new Pose2d(),
                this.context.getStateStdDevs(),
                this.context.getVisionMeasurementStdDevs());

        this.addChild(frontLeftSwerveModule.getName(), frontLeftSwerveModule);
        this.addChild(frontRightSwerveModule.getName(), frontRightSwerveModule);
        this.addChild(rearLeftSwerveModule.getName(), rearLeftSwerveModule);
        this.addChild(rearRightSwerveModule.getName(), rearRightSwerveModule);
        this.addChild("navx", this.navXSensorModule);
    }

    public double getMaxSpeed() {
        return this.context.getMaxSpeed();
    }

    public double getMaxTurnAngularSpeed() {
        return this.context.getMaxTurnAngularSpeed();
    }

    public boolean isFieldRelativeEnable() {
        return this.fieldRelativeEnable;
    }

    public void setFieldRelativeEnable(boolean enable) {
        this.fieldRelativeEnable = enable;
    }

    /**
     * Gets our current position in meters on the field.
     *
     * @return A current position on the field.
     *         <p>
     *         <pi> A translation2d (X and Y on the field) -> {@link #swerveDriveKinematics} + A rotation2d (Rot X and Y
     *         on the field) -> {@link #navXSensorModule}
     */
    private Pose2d getPose2d() {
        return this.swerveDriveOdometry.getPoseMeters();
    }

    private Pose2d getPose2dEstimator() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Tells our modules what speed to go to
     */
    private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.DRIVE_MAX_SPEED);
        this.frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        this.frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
        this.rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
        this.rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Tells our wheels to go to the Wheel Locking position (0 m/s, forming an X)
     */
    private void lockWheels() {
        this.rearLeftSwerveModule.setDesiredState(this.context.getFullStopAt135Degrees());
        this.frontLeftSwerveModule.setDesiredState(this.context.getFullStopAt45Degrees());
        this.rearRightSwerveModule.setDesiredState(this.context.getFullStopAt45Degrees());
        this.frontRightSwerveModule.setDesiredState(this.context.getFullStopAt135Degrees());
    }

    /**
     * Gets the Position of the four SwerveModules.
     *
     * <p>
     * This gets the encoder in the motor (drive) and the encoder on the swerve module.
     */
    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeftSwerveModule.getModulePosition(),
            this.frontRightSwerveModule.getModulePosition(),
            this.rearLeftSwerveModule.getModulePosition(),
            this.rearRightSwerveModule.getModulePosition()
        };
    }

    private void driveRobotRelative(final ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        this.setModuleStates(this.swerveDriveKinematics.toSwerveModuleStates(targetSpeeds));
    }

    /**
     * Updates our current Odometry
     */
    private void updateOdometry() {
        this.swerveDriveOdometry.update(this.navXSensorModule.getRotation2d(), this.getSwerveModulePositions());
    }

    private void updatePoseEstimatorOdometry() {
        this.swerveDrivePoseEstimator.update(this.getHeading(), this.getSwerveModulePositions());

        boolean doRejectUpdate = false;
        if (!this.context.isLimelightMegaTag2AlgorithmEnabled()) {
            LimelightHelpers.PoseEstimate mt1 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(Limelights.LIMELIGHT_FRONT_LEFT);

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                LimelightHelpers.RawFiducial rawFiducial0 = mt1.rawFiducials[0];
                if (rawFiducial0.ambiguity > this.context.getRawFiducialAmbiguityThreshold()
                        || rawFiducial0.distToCamera > this.context.getRawFiducialDistanceToCameraThreshold()) {
                    doRejectUpdate = true;
                }
            } else if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                this.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                this.swerveDrivePoseEstimator.addVisionMeasurement(mt1.pose, mt1.timestampSeconds);
            }
        } else {
            LimelightHelpers.SetRobotOrientation(
                    Limelights.LIMELIGHT_FRONT_LEFT,
                    this.swerveDrivePoseEstimator
                            .getEstimatedPosition()
                            .getRotation()
                            .getDegrees(),
                    this.navXSensorModule.getRate(),
                    this.navXSensorModule.getPitch(),
                    0,
                    this.navXSensorModule.getRoll(),
                    0);

            LimelightHelpers.PoseEstimate mt2 =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Limelights.LIMELIGHT_FRONT_LEFT);

            // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            if (Math.abs(navXSensorModule.getRate()) > this.context.getAngularVelocityThreshold()) {
                doRejectUpdate = true;
            }

            if (mt2 == null || mt2.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                this.swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                this.swerveDrivePoseEstimator.addVisionMeasurement(mt2.pose, mt2.timestampSeconds);
            }
        }
    }

    /**
     * Reset's the Robots Odometry using the Gyro's Current Rotational Position
     *
     * @param pose2d
     */
    public void resetOdometry(final Pose2d pose2d) {
        this.swerveDriveOdometry.resetPosition(
                this.navXSensorModule.getRotation2d(), this.getSwerveModulePositions(), pose2d);

        if (RobotBase.isSimulation()) {
            // Keep simulated gyro aligned with newPose
            this.simYaw = pose2d.getRotation();
        }
    }

    /**
     * Utility to obtain the heading of the robot in real or sim
     *
     * @return The heading
     */
    private Rotation2d getHeading() {
        if (RobotBase.isSimulation()) {
            return this.simYaw;
        } else {
            return this.navXSensorModule.getRotation2d();
        }
    }

    /**
     * Converts raw module states into chassis speeds
     *
     * @return chassisSpeeds --> A reading of the speed in m/s our robot is going.
     */
    private ChassisSpeeds getChassisSpeeds() {
        return this.swerveDriveKinematics.toChassisSpeeds(this.getSwerveModuleStates());
    }

    /**
     * This command gets the 4 individual SwerveModule States, and groups it into 1 array. <pi> Used for getting our
     * chassis (robots) speed.
     *
     * @return 4 different SwerveModuleStates
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    private SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[] {
            this.frontLeftSwerveModule.getModuleState(),
            this.frontRightSwerveModule.getModuleState(),
            this.rearLeftSwerveModule.getModuleState(),
            this.rearRightSwerveModule.getModuleState()
        };
    }

    private Pose2d refreshGoalPose2d() {
        // TODO: Why is this hard-coded to PositionsRed?
        this.goalPose = this.getPose2dEstimator().nearest(Constants.Poses.PositionsRed);
        return this.goalPose;
    }

    /**
     * Stops all the motors on the SwerveModules
     */
    public void stopModules() {
        this.frontLeftSwerveModule.stopMotors();
        this.frontRightSwerveModule.stopMotors();
        this.rearLeftSwerveModule.stopMotors();
        this.rearRightSwerveModule.stopMotors();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed
     *            Speed of the robot in the x direction (forward).
     * @param ySpeed
     *            Speed of the robot in the y direction (sideways).
     * @param rot
     *            Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        SmartDashboard.putNumber(getName() + "/Command/X Speed", xSpeed);
        SmartDashboard.putNumber(getName() + "/Command/Y Speed", ySpeed);
        SmartDashboard.putNumber(getName() + "/Command/Rot Speed", rot);
        SmartDashboard.putBoolean(getName() + "/Command/RobotRelative", this.fieldRelativeEnable);
        Rotation2d robotRotation =
                new Rotation2d(navXSensorModule.getRotation2d().getRadians());
        this.desiredStates = this.swerveDriveKinematics.toSwerveModuleStates(
                this.fieldRelativeEnable
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        if (!this.wheelLock) {
            this.setModuleStates(this.desiredStates);
        } else {
            this.lockWheels();
        }
    }

    /**
     * Runnable Command.
     *
     * <p>
     * Tells the Wheels when to stop or not based off of a boolean variable named {@link #wheelLock}.
     *
     * <p>
     * Used in drive Method
     */
    public Command toggleWheelLockCommand() {
        return this.runOnce(() -> this.wheelLock = !this.wheelLock);
    }

    /**
     * Runnable Command.
     *
     * <p>
     * Tells the Gyro to reset its heading/which way its facing.
     *
     * <p>
     * Used in drive Method.
     */
    public Command resetNavXSensorModule() {
        return this.runOnce(this.navXSensorModule::reset);
    }

    /**
     * This is a runnable command.
     * <li>This resets the gyro's position.
     * <li>This is needed for Auto, Limelight, and the DriveTrain.
     *
     * @return Pose2d
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    public Command getResetOdometryCommand() {
        return this.runOnce(() -> this.resetOdometry(new Pose2d()));
    }

    /**
     * This is a runnable command.
     * <li>This toggles field relative on and off.
     * <li>If
     *
     * @return Pose2d
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    public Command getToggleFieldRelativeCommand() {
        return this.runOnce(() -> this.fieldRelativeEnable = !this.fieldRelativeEnable);
    }

    /**
     * Runnable Command. Runs the {@link #stopModules()} Command.
     */
    public Command getStopModulesCommand() {
        return this.run(this::stopModules);
    }

    @Override
    public void periodic() {
        this.updateOdometry();
        this.updatePoseEstimatorOdometry();
        super.periodic();
        this.field.setRobotPose(this.getPose2dEstimator());
        this.desiredStatePublisher.set(this.desiredStates);
        this.currentStatePublisher.set(this.getSwerveModuleStates());
        this.currentSpeedsPublisher.set(this.getChassisSpeeds());
        this.currentPosePublisher.set(this.getPose2d());
        this.currentRotPublisher.set(this.getPose2d().getRotation());
        this.currentPoseEstimatorPublisher.set(this.getPose2dEstimator());
        this.goalPosePublisher.set(this.goalPose);
    }

    @Override
    public void simulationPeriodic() {
        // 1. Compute elapsed time since last loop
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - lastSimTime;
        this.lastSimTime = currentTime;

        // 2. Update each simulated swerve module using stored percent outputs
        frontLeftSwerveModuleSim.setDriveVoltage(frontLeftSwerveModule.getLastDrivePercent() * 12.0);
        frontLeftSwerveModuleSim.setTurnVoltage(frontLeftSwerveModule.getLastTurnPercent() * 12.0);
        frontLeftSwerveModuleSim.update(dt);

        frontRightSwerveModuleSim.setDriveVoltage(frontRightSwerveModule.getLastDrivePercent() * 12.0);
        frontRightSwerveModuleSim.setTurnVoltage(frontRightSwerveModule.getLastTurnPercent() * 12.0);
        frontRightSwerveModuleSim.update(dt);

        rearLeftSwerveModuleSim.setDriveVoltage(rearLeftSwerveModule.getLastDrivePercent() * 12.0);
        rearLeftSwerveModuleSim.setTurnVoltage(rearLeftSwerveModule.getLastTurnPercent() * 12.0);
        rearLeftSwerveModuleSim.update(dt);

        rearRightSwerveModuleSim.setDriveVoltage(rearRightSwerveModule.getLastDrivePercent() * 12.0);
        rearRightSwerveModuleSim.setTurnVoltage(rearRightSwerveModule.getLastTurnPercent() * 12.0);
        rearRightSwerveModuleSim.update(dt);

        // 3. Build simulated module states for kinematics
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(
                    this.frontLeftSwerveModuleSim.getWheelSpeedMetersPerSecond(),
                    this.frontLeftSwerveModuleSim.getTurnAngle()),
            new SwerveModuleState(
                    this.frontRightSwerveModuleSim.getWheelSpeedMetersPerSecond(),
                    this.frontRightSwerveModuleSim.getTurnAngle()),
            new SwerveModuleState(
                    this.rearLeftSwerveModuleSim.getWheelSpeedMetersPerSecond(),
                    this.rearLeftSwerveModuleSim.getTurnAngle()),
            new SwerveModuleState(
                    this.rearRightSwerveModuleSim.getWheelSpeedMetersPerSecond(),
                    this.rearRightSwerveModuleSim.getTurnAngle())
        };

        // 4. Convert to chassis speeds and integrate heading
        ChassisSpeeds chassisSpeeds = this.swerveDriveKinematics.toChassisSpeeds(states);
        Rotation2d delta = new Rotation2d(chassisSpeeds.omegaRadiansPerSecond * dt);
        this.simYaw = this.simYaw.rotateBy(delta);

        // 5. Update pose estimator with sim yaw and module positions
        this.swerveDrivePoseEstimator.update(this.simYaw, new SwerveModulePosition[] {
            this.frontLeftSwerveModuleSim.getPosition(),
            this.frontRightSwerveModuleSim.getPosition(),
            this.rearLeftSwerveModuleSim.getPosition(),
            this.rearRightSwerveModuleSim.getPosition()
        });

        // 6. Push pose to Field2d for visualization
        this.field.setRobotPose(swerveDrivePoseEstimator.getEstimatedPosition());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("Odometry/Pose/X", () -> this.getPose2d().getX(), null);
        builder.addDoubleProperty("Odometry/Pose/Y", () -> this.getPose2d().getY(), null);
        builder.addDoubleProperty(
                "Odometry/Pose/Rot", () -> this.getPose2d().getRotation().getDegrees(), null);
        builder.addDoubleProperty("Odometry/ChassisSpeeds/X", () -> this.getChassisSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty("Odometry/ChassisSpeeds/Y", () -> this.getChassisSpeeds().vyMetersPerSecond, null);
        builder.addDoubleProperty(
                "Odometry/ChassisSpeeds/Rot",
                () -> Units.radiansToDegrees(this.getChassisSpeeds().omegaRadiansPerSecond),
                null);
        builder.addDoubleProperty(
                "Odometry/navx/Orientation",
                () -> this.navXSensorModule.getRotation2d().getDegrees(),
                null);
        builder.addBooleanProperty(
                "FieldRelativeEnabled",
                () -> this.fieldRelativeEnable,
                (boolean fre) -> this.fieldRelativeEnable = fre);
        builder.addDoubleProperty(
                "EstimatedOdometry/Pose/X", () -> this.getPose2dEstimator().getX(), null);
        builder.addDoubleProperty(
                "EstimatedOdometry/Pose/Y", () -> this.getPose2dEstimator().getY(), null);
        builder.addDoubleProperty(
                "EstimatedOdometry/Pose/Rot",
                () -> this.getPose2dEstimator().getRotation().getDegrees(),
                null);
        builder.addDoubleProperty("GOALPOSE/X", () -> this.refreshGoalPose2d().getX(), null);
        builder.addDoubleProperty("GOALPOSE/Y", () -> this.refreshGoalPose2d().getY(), null);
        builder.addDoubleProperty(
                "GOALPOSE/ROT", () -> this.refreshGoalPose2d().getRotation().getRadians(), null);

        SmartDashboard.putData("DriveTrain/" + this.frontLeftSwerveModule.getName(), this.frontLeftSwerveModule);
        SmartDashboard.putData("DriveTrain/" + this.frontRightSwerveModule.getName(), this.frontRightSwerveModule);
        SmartDashboard.putData("DriveTrain/" + this.rearLeftSwerveModule.getName(), this.rearLeftSwerveModule);
        SmartDashboard.putData("DriveTrain/" + this.rearRightSwerveModule.getName(), this.rearRightSwerveModule);
        SmartDashboard.putData("field", this.field);

        builder.addDoubleProperty("GYRO ANGLE", this.navXSensorModule::getAngle, null);
        SmartDashboard.putData("NAVX DATA", this.navXSensorModule);
        builder.addDoubleProperty(
                "NAVX ROTATION", () -> this.navXSensorModule.getRotation2d().getDegrees(), null);
        builder.addDoubleProperty("NAVX AngleAdjustment", this.navXSensorModule::getAngleAdjustment, null);
    }
}
