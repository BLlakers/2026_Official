// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

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
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.limelight.LimelightHelpers;

import java.util.Optional;

/**
 * Represents a swerve drive style drivetrain. In here, we initialize our swerve modules
 * (example -> {@link #frontLeftSwerveModule}), Get input from autonomous and initialize our
 * odometry -> {@link #swerveDriveOdometry}. Various other DriveTrain Related thing are initialized here too.
 */
public class Drivetrain extends SubsystemBase {

    private final DrivetrainSettings settings;

    private final Field2d field = new Field2d();

    private final SwerveDriveKinematics swerveDriveKinematics;

    // The gyro object. Gyro gives the robots rotation/ where the robot is pointed.
    private final AHRS navXSensorModule;

    public final SwerveModule frontRightSwerveModule;

    public final SwerveModule frontLeftSwerveModule;

    public final SwerveModule rearLeftSwerveModule;

    public final SwerveModule rearRightSwerveModule;

    private final SwerveDriveOdometry swerveDriveOdometry;

    private final SwerveDrivePoseEstimator swerveDrivePoseEstimator;

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

    private SwerveModuleState[] DesiredStates;

    private SwerveModuleState[] CurrentStates; // TODO: Unused? Eliminate

    /**
     * Instantiates a new Drivetrain with default {@link DrivetrainSettings}
     */
    public Drivetrain() {
        this(DrivetrainSettings.defaults());
    }

    /**
     * Instantiates a new Drivetrain subsystem with the specified settings
     *
     * @param settings The DrivetrainSettings to apply to this instance
     */
    public Drivetrain(final DrivetrainSettings settings) {

        this.settings = settings;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();

        this.desiredStatePublisher = nti.getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();

        this.currentStatePublisher = nti.getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();

        this.currentSpeedsPublisher = nti.getStructTopic("CurrentSpeed", ChassisSpeeds.struct).publish();

        this.currentPosePublisher = nti.getStructTopic("CurrentPose", Pose2d.struct).publish();

        this.currentPoseEstimatorPublisher = nti.getStructTopic("CurrentPoseEstimator", Pose2d.struct).publish();

        this.goalPosePublisher = nti.getStructTopic("GoalPoseEstimator", Pose2d.struct).publish();

        this.currentRotPublisher = nti.getStructTopic("CurrentRot", Rotation2d.struct).publish();

        this.navXSensorModule = new AHRS(AHRS.NavXComType.kMXP_SPI);

        this.settings.getRobotConfig().ifPresentOrElse(robotConfig -> {
            AutoBuilder.configure(
                    this::getPose2d, // Robot pose supplier NEEDS TO BE POSE2D IF WE ARE USING OLD LIMELIGHT WAY TODO
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose) NEEDS TO BE RESETPOSE2D IF WE ARE USING OLD LIMELIGHT
                    this::getChassisSpeeds,
                    this::driveRobotRelative, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also, optionally outputs individual module feedforwards
                    new PPHolonomicDriveController(
                            new PIDConstants(this.settings.getLateralMovementPIDSettings().p()),
                            new PIDConstants(this.settings.getRotationPIDSettings().p())),
                    //new PPHolonomicDriveController(Translation,Rotation), //PPHolonomicDriveController(Translation, Rotation, .2),
                    robotConfig,// The robot configuration
                    () -> {
                        // Boolean supplier that controls when the path will be mirrored for the red alliance
                        // This will flip the path being followed to the red side of the field.
                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                        Optional<Alliance> alliance = DriverStation.getAlliance();
                        return alliance.filter(value -> value == Alliance.Red).isPresent();
                    },
                    this); // Reference to this subsystem to set requirements
        }, () -> {
            throw new RuntimeException("Unable to obtain RobotConfig during Drivetrain creation." +
                    "Path Planning will be fail!");
        });

        this.swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.Drive.SMFrontLeftLocation,
                Constants.Drive.SMFrontRightLocation,
                Constants.Drive.SMBackLeftLocation,
                Constants.Drive.SMBackRightLocation);

        this.frontRightSwerveModule = new SwerveModule(this.settings.getFrontRightSwerveModuleSettings());
        this.frontLeftSwerveModule = new SwerveModule(this.settings.getFrontLeftSwerveModuleSettings());
        this.rearLeftSwerveModule = new SwerveModule(this.settings.getRearLeftSwerveModuleSettings());
        this.rearRightSwerveModule = new SwerveModule(this.settings.getRearRightSwerveModuleSettings());

        // initializes odometry
        swerveDriveOdometry =
                new SwerveDriveOdometry(
                        this.swerveDriveKinematics, this.navXSensorModule.getRotation2d(), getSwerveModulePositions());

        swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                this.swerveDriveKinematics,
                this.navXSensorModule.getRotation2d(),
                new SwerveModulePosition[]{
                        frontLeftSwerveModule.getModulePosition(),
                        frontRightSwerveModule.getModulePosition(),
                        rearLeftSwerveModule.getModulePosition(),
                        rearRightSwerveModule.getModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        addChild(frontLeftSwerveModule.getName(), frontLeftSwerveModule);
        addChild(frontRightSwerveModule.getName(), frontRightSwerveModule);
        addChild(rearLeftSwerveModule.getName(), rearLeftSwerveModule);
        addChild(rearRightSwerveModule.getName(), rearRightSwerveModule);

        addChild("navx", this.navXSensorModule);
    }

    public double getMaxSpeed() {
        return this.settings.getMaxSpeed();
    }

    public double getMaxTurnAngularSpeed() {
        return this.settings.getMaxTurnAngularSpeed();
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
     * <p><pi> A translation2d (X and Y on the field) -> {@link #swerveDriveKinematics} + A rotation2d (Rot
     * X and Y on the field) -> {@link #navXSensorModule}
     */
    public Pose2d getPose2d() {
        return swerveDriveOdometry.getPoseMeters();
    }

    public Pose2d getPose2dEstimator() {
        return swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the Position of the four SwerveModules.
     *
     * <p>This gets the encoder in the motor (drive) and the encoder on the swerve module.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition frontLeftPosition = frontLeftSwerveModule.getModulePosition();
        SwerveModulePosition frontRightPosition = frontRightSwerveModule.getModulePosition();
        SwerveModulePosition backLeftPosition = rearLeftSwerveModule.getModulePosition();
        SwerveModulePosition backRightPosition = rearRightSwerveModule.getModulePosition();
        return new SwerveModulePosition[]{
                frontLeftPosition,
                frontRightPosition,
                backLeftPosition,
                backRightPosition
        };
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot) {

        SmartDashboard.putNumber(getName() + "/Command/X Speed", xSpeed);
        SmartDashboard.putNumber(getName() + "/Command/Y Speed", ySpeed);
        SmartDashboard.putNumber(getName() + "/Command/Rot Speed", rot);
        SmartDashboard.putBoolean(getName() + "/Command/RobotRelative", this.fieldRelativeEnable);

        var alliance = DriverStation.getAlliance();
        Rotation2d robotRotation;

        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //   robotRotation = new Rotation2d(navx.getRotation2d().getRadians() + Rotation2d.fromDegrees(180).getRadians());
        // } else {
        robotRotation = new Rotation2d(navXSensorModule.getRotation2d().getRadians());
        // }

        // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
        DesiredStates =
                swerveDriveKinematics.toSwerveModuleStates(
                        this.fieldRelativeEnable
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

        if (!wheelLock) {
            setModuleStates(DesiredStates);
        } else {
            WheelLock();
        }
    }

    /**
     * Tells our modules what speed to go to
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.DRIVE_MAX_SPEED);
        frontLeftSwerveModule.setDesiredState(swerveModuleStates[0]);
        frontRightSwerveModule.setDesiredState(swerveModuleStates[1]);
        rearLeftSwerveModule.setDesiredState(swerveModuleStates[2]);
        rearRightSwerveModule.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Tells our wheels to go to the Wheel Locking position (0 m/s, forming an X)
     */
    public void WheelLock() {
        rearLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
        frontLeftSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
        rearRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
        frontRightSwerveModule.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
    }

    @Override
    public void periodic() {
        updateOdometry();
        updatePoseEstimatorOdometry();
        super.periodic();
        this.field.setRobotPose(getPose2dEstimator());
        desiredStatePublisher.set(DesiredStates);
        currentStatePublisher.set(getSwerveModuleStates());
        currentSpeedsPublisher.set(getChassisSpeeds());
        currentPosePublisher.set(getPose2d());
        currentRotPublisher.set(getPose2d().getRotation());
        currentPoseEstimatorPublisher.set(getPose2dEstimator());
        goalPosePublisher.set(goalPose);
    }

    /**
     * Runnable Command.
     *
     * <p>Tells the Wheels when to stop or not based off of a boolean varible named {@link
     * #wheelLock}.
     *
     * <p>Used in drive Method
     */
    public Command WheelLockCommand() {

        return this.runOnce(
                () -> {

                    // one-time action goes here
                    // WP - Add code here to toggle the gripper solenoid
                    if (wheelLock == true) {
                        wheelLock = false;
                    } else if (wheelLock == false) {
                        wheelLock = true;
                    }
                });
    }

    /**
     * Runnable Command.
     *
     * <p>Tells the Gyro to reset its heading/which way its facing.
     *
     * <p>Used in drive Method.
     */
    public Command ZeroGyro() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.

        return this.runOnce(
                () -> {
                    navXSensorModule.reset();
                });
    }

    public Command SetGyroAdjustmentAngle() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.

        return this.runOnce(
                () -> {
                    navXSensorModule.setAngleAdjustment(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-frl").pose.getRotation().getDegrees());
                });
    }


    /**
     * Tells the robot to drive based of off a given velocity.
     *
     * <p>Used for Autonomous.
     *
     * @param chassisSpeed (ChassisSpeeds) - this is the desired velocity we would like to drive the
     *                     robot.
     */
    public void driveChassisSpeeds(ChassisSpeeds chassisSpeed) {
        drive(
                chassisSpeed.vxMetersPerSecond,
                chassisSpeed.vyMetersPerSecond,
                chassisSpeed.omegaRadiansPerSecond);
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose2dEstimator().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] targetStates = swerveDriveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }


    /**
     * Resets the Position of the Odometer, given our Current position.
     *
     * @param pose2d - The current position of the robot on the field. This is a {@link
     *               #resetOdometry(Pose2d)}
     */
    public void resetPose(Pose2d pose2d) {
        resetOdometry(pose2d);
    }

    public void resetPoseEstimator(Pose2d pose2d) {
        swerveDrivePoseEstimator.resetPosition(navXSensorModule.getRotation2d(), getSwerveModulePositions(), pose2d);
    }

    /**
     * Reset's the Robots Odometry using the Gyro's Current Rotational Position
     *
     * @param pose2d
     */
    public void resetOdometry(Pose2d pose2d) {
        swerveDriveOdometry.resetPosition(navXSensorModule.getRotation2d(), getSwerveModulePositions(), pose2d);
    }

    /**
     * Converts raw module states into chassis speeds
     *
     * @return chassisSpeeds --> A reading of the speed in m/s our robot is going.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return swerveDriveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    /**
     * This command gets the 4 individual SwerveModule States, and groups it into 1 array. <pi> Used
     * for getting our chassis (robots) speed.
     *
     * @return 4 different SwerveModuleStates
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    public SwerveModuleState[] getSwerveModuleStates() {
        return new SwerveModuleState[]{
                frontLeftSwerveModule.getModuleState(),
                frontRightSwerveModule.getModuleState(),
                rearLeftSwerveModule.getModuleState(),
                rearRightSwerveModule.getModuleState()
        };
    }

    /**
     * This is a runnable command.
     * <li>This resets the gyro's position.
     * <li>This is needed for Auto, Limelight, and the DriveTrain.
     *
     * @return Pose2d
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    public Command resetPose2d() {
        return this.runOnce(
                () -> {
                    resetPose(new Pose2d());
                });
    }

    public Pose2d getGoal() {
        goalPose = getPose2dEstimator().nearest(Constants.Poses.PositionsRed);
        // Goal end velocity in meters/sec
        return goalPose;
    }

    /**
     * This is a runnable command.
     * <li>This toggles field relative on and off.
     * <li>If
     *
     * @return Pose2d
     * @author Jared Forchheimer, Dimitri Lezcano
     */
    public Command toggleFieldRelativeEnable() {

        return this.runOnce(
                () -> {
                    // System.out.println("I am Here");
                    // one-time action goes here
                    // WP - Add code here to toggle the gripper solenoid
                    if (this.fieldRelativeEnable) {
                        fieldRelativeEnable = false;
                        // System.out.println("I am Here 2");
                    } else {
                        fieldRelativeEnable = true;
                        // System.out.println("I am Here 3");
                    }
                });
    }

    public void SetFieldRelativeEnable(boolean fieldRelative) {
        fieldRelativeEnable = fieldRelative;
    }

    /**
     * Updates our current Odometry
     */
    public void updateOdometry() {
        swerveDriveOdometry.update(navXSensorModule.getRotation2d(), getSwerveModulePositions());
    }

    public Command resetPoseEstimatorCmd() {
        return this.runOnce(() -> resetPoseEstimator(getPose2dEstimator()));
    }

    public void updatePoseEstimatorOdometry() {
        swerveDrivePoseEstimator.update(
                navXSensorModule.getRotation2d(),
                new SwerveModulePosition[]{
                        frontLeftSwerveModule.getModulePosition(),
                        frontRightSwerveModule.getModulePosition(),
                        rearLeftSwerveModule.getModulePosition(),
                        rearRightSwerveModule.getModulePosition()
                });


        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (!useMegaTag2) {
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-frl");

            if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                if (mt1.rawFiducials[0].ambiguity > .7) {
                    doRejectUpdate = true;
                }
                if (mt1.rawFiducials[0].distToCamera > 3) {
                    doRejectUpdate = true;
                }
            }
            if (mt1.tagCount == 0) {
                doRejectUpdate = true;
            }

            if (!doRejectUpdate) {
                swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                swerveDrivePoseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2) {
            LimelightHelpers.SetRobotOrientation("limelight-frl", swerveDrivePoseEstimator.getEstimatedPosition().getRotation().getDegrees(), navXSensorModule.getRate(), navXSensorModule.getPitch(), 0, navXSensorModule.getRoll(), 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-frl");
            if (Math.abs(navXSensorModule.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2 == null) {
                doRejectUpdate = true;
            } else if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {

                swerveDrivePoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                swerveDrivePoseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    }

    /**
     * Stops all the motors on the SwerveModules
     */
    public void stopModules() {
        frontLeftSwerveModule.stopMotors();
        frontRightSwerveModule.stopMotors();
        rearLeftSwerveModule.stopMotors();
        rearRightSwerveModule.stopMotors();
    }

    /**
     * Runnable Command. Runs the {@link #stopModules()} Command.
     */
    public Command Break() {
        return this.run(this::stopModules);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("Odometry/Pose/X", () -> getPose2d().getX(), null);
        builder.addDoubleProperty("Odometry/Pose/Y", () -> getPose2d().getY(), null);
        builder.addDoubleProperty(
                "Odometry/Pose/Rot", () -> getPose2d().getRotation().getDegrees(), null);
        builder.addDoubleProperty(
                "Odometry/ChassisSpeeds/X", () -> getChassisSpeeds().vxMetersPerSecond, null);
        builder.addDoubleProperty(
                "Odometry/ChassisSpeeds/Y", () -> getChassisSpeeds().vyMetersPerSecond, null);
        builder.addDoubleProperty(
                "Odometry/ChassisSpeeds/Rot",
                () -> Units.radiansToDegrees(getChassisSpeeds().omegaRadiansPerSecond),
                null);
        builder.addDoubleProperty(
                "Odometry/navx/Orientation", () -> navXSensorModule.getRotation2d().getDegrees(), null);
        builder.addBooleanProperty(
                "FieldRelativeEnabled",
                () -> this.fieldRelativeEnable,
                (boolean fre) -> fieldRelativeEnable = fre);
        builder.addDoubleProperty("EstimatedOdometry/Pose/X", () -> getPose2dEstimator().getX(), null);
        builder.addDoubleProperty("EstimatedOdometry/Pose/Y", () -> getPose2dEstimator().getY(), null);
        builder.addDoubleProperty(
                "EstimatedOdometry/Pose/Rot", () -> getPose2dEstimator().getRotation().getDegrees(), null);
        builder.addDoubleProperty("GOALPOSE/X", () -> getGoal().getX(), null);
        builder.addDoubleProperty("GOALPOSE/Y", () -> getGoal().getY(), null);
        builder.addDoubleProperty("GOALPOSE/ROT", () -> getGoal().getRotation().getRadians(), null);
        SmartDashboard.putData("DriveTrain/" + frontLeftSwerveModule.getName(), frontLeftSwerveModule);
        SmartDashboard.putData("DriveTrain/" + frontRightSwerveModule.getName(), frontRightSwerveModule);
        SmartDashboard.putData("DriveTrain/" + rearLeftSwerveModule.getName(), rearLeftSwerveModule);
        SmartDashboard.putData("DriveTrain/" + rearRightSwerveModule.getName(), rearRightSwerveModule);
        SmartDashboard.putData("field", field);
        builder.addDoubleProperty("GYRO ANGLE", () -> navXSensorModule.getAngle(), null);
        SmartDashboard.putData("NAVX DATA", navXSensorModule);
        builder.addDoubleProperty("NAVX ROTATION", () -> navXSensorModule.getRotation2d().getDegrees(), null);
        builder.addDoubleProperty("NAVX AngleAdjustment", () -> navXSensorModule.getAngleAdjustment(), null);
    }
}
