// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;
import com.studica.frc.AHRS;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.RobotContainer;
import frc.robot.support.RobotVersion;
import frc.robot.support.limelight.LimelightHelpers;

import java.util.List;

/**
 * Represents a swerve drive style drivetrain.
 */
public class DriveTrain extends SubsystemBase {
    Field2d field;
    public SwerveDriveKinematics m_kinematics =
            new SwerveDriveKinematics(
                    Constants.Drive.SMFrontLeftLocation,
                    Constants.Drive.SMFrontRightLocation,
                    Constants.Drive.SMBackLeftLocation,
                    Constants.Drive.SMBackRightLocation);
    public PIDConstants LeftToRight = new PIDConstants(3); //
    public PIDConstants Rotation = new PIDConstants(3);
    public boolean m_WheelLock = false;
    public boolean m_FieldRelativeEnable = true;
    Pose2d goalPose;
    public static final double kMaxSpeed =
            Units.feetToMeters(12.5); // WP this seemed to work don't know why // 3.68
    // meters per second or 12.1
    // ft/s (max speed of SDS Mk3 with Neo motor) // TODO KMaxSpeed needs to go with
    // enum
    public static final double kMaxAngularSpeed =
            Units.rotationsPerMinuteToRadiansPerSecond(
                    Constants.Conversion.NeoMaxSpeedRPM / Constants.Conversion.TurnGearRatio); // 1/2
    // rotation
    // per
    // second
    public static final double kMaxTurnAngularSpeed =
            kMaxSpeed / Constants.Drive.SMBackLeftLocation.getNorm(); // 1/2
    // rotation
    // per
    // second
    public static final double kModuleMaxAngularAcceleration =
            Math.PI / 3; // what is this used for again?

    // creates a gyro object. Gyro gives the robots rotation/ where the robot is
    // pointed.
    public final AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI);

    // Creates each swerve module. Swerve modules have a turning and drive motor + a
    // turning and drive encoder.
    public final SwerveModule m_frontRight;
    public final SwerveModule m_frontLeft;
    public final SwerveModule m_backLeft;
    public final SwerveModule m_backRight;

    // Creates an odometry object. Odometry tells the robot its position on the
    // field.
    public RobotConfig config;

    {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    private final SwerveDriveOdometry m_odometry;

    private final SwerveDrivePoseEstimator m_poseEstimator;


    // Constructor
    /**
     * Our driveTrain Constructor.
     *
     * <p>In here, we initialize our swerve modules (example -> {@link #m_frontLeft}), Get input from
     * autonomous and initialize our odometry -> {@link #m_odometry}.
     *
     * <p>Various other DriveTrain Related thing are initalized here too.
     *
     * @param RobotVersion
     */
    SwerveModuleState[] DesiredStates;
    SwerveModuleState[] CurrentStates;
    StructArrayPublisher<SwerveModuleState> DesiredStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("DesiredStates", SwerveModuleState.struct).publish();
    StructArrayPublisher<SwerveModuleState> CurrentStatePublisher = NetworkTableInstance.getDefault()
            .getStructArrayTopic("CurrentStates", SwerveModuleState.struct).publish();
    StructPublisher<ChassisSpeeds> CurrentSpeedsPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CurrentSpeed", ChassisSpeeds.struct).publish();
    StructPublisher<Pose2d> CurrentPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CurrentPose", Pose2d.struct).publish();
    StructPublisher<Pose2d> CurrentPoseEstimatorPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CurrentPoseEstimator", Pose2d.struct).publish();
    StructPublisher<Pose2d> GoalPosePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("GoalPoseEstimator", Pose2d.struct).publish();

    StructPublisher<Rotation2d> CurrentRotPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("CurrentRot", Rotation2d.struct).publish();

    public DriveTrain(RobotVersion version) {
        AutoBuilder.configure(
                this::getPose2d, // Robot pose supplier NEEDS TO BE POSE2D IF WE ARE USING OLD LIMELIGHT WAY TODO
                this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose) NEEDS TO BE RESETPOSE2D IF WE ARE USING OLD LIMELIGHT
                this::getChassisSpeeds,
                (speeds) -> driveRobotRelative(speeds), // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController(LeftToRight, Rotation),
                //new PPHolonomicDriveController(Translation,Rotation), //PPHolonomicDriveController(Translation, Rotation, .2),
                config,// The robot configuration
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this); // Reference to this subsystem to set requirements

        // sets our wanted offsets. Varies between 2023 and 2024.
        double flTurnOffset = 0, frTurnOffset = 0, blTurnOffset = 0, brTurnOffset = 0;
        if (Constants.defaultRobotVersion == RobotVersion.v2023) {
            flTurnOffset = Constants.RobotVersion2023.flTurnEncoderOffset;
            frTurnOffset = Constants.RobotVersion2023.frTurnEncoderOffset;
            blTurnOffset = Constants.RobotVersion2023.blTurnEncoderOffset;
            brTurnOffset = Constants.RobotVersion2023.brTurnEncoderOffset;
        } else if (Constants.defaultRobotVersion == RobotVersion.v2025) {
            flTurnOffset = Constants.RobotVersion2025.flTurnEncoderOffset;
            frTurnOffset = Constants.RobotVersion2025.frTurnEncoderOffset;
            blTurnOffset = Constants.RobotVersion2025.blTurnEncoderOffset;
            brTurnOffset = Constants.RobotVersion2025.brTurnEncoderOffset;
        }


        m_frontRight =
                new SwerveModule(
                        Constants.Port.frDriveMtrC,
                        Constants.Port.frSteerMtrC,
                        Constants.Port.frTurnEncoderDIOC,
                        frTurnOffset);
        m_frontLeft =
                new SwerveModule(
                        Constants.Port.flDriveMtrC,
                        Constants.Port.flSteerMtrC,
                        Constants.Port.flTurnEncoderDIOC,
                        flTurnOffset);
        m_backLeft =
                new SwerveModule(
                        Constants.Port.blDriveMtrC,
                        Constants.Port.blSteerMtrC,
                        Constants.Port.blTurnEncoderDIOC,
                        blTurnOffset);
        m_backRight =
                new SwerveModule(
                        Constants.Port.brDriveMtrC,
                        Constants.Port.brSteerMtrC,
                        Constants.Port.brTurnEncoderDIOC,
                        brTurnOffset); // 0.05178

        m_frontLeft.setName("Swerve Module/Front Left");
        m_frontRight.setName("Swerve Module/Front Right");
        m_backLeft.setName("Swerve Module/Back Left");
        m_backRight.setName("Swerve Module/Back Right");
        // initializes odometry
        m_odometry =
                new SwerveDriveOdometry(
                        this.m_kinematics, navx.getRotation2d(), getSwerveModulePositions());

        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                navx.getRotation2d(),
                new SwerveModulePosition[]{
                        m_frontLeft.getModulePosition(),
                        m_frontRight.getModulePosition(),
                        m_backLeft.getModulePosition(),
                        m_backRight.getModulePosition()
                },
                new Pose2d(),
                VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
                VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));

        addChild(m_frontLeft.getName(), m_frontLeft);
        addChild(m_frontRight.getName(), m_frontRight);
        addChild(m_backLeft.getName(), m_backLeft);
        addChild(m_backRight.getName(), m_backRight);

        addChild("navx", navx);
        field = new Field2d();
    }

    /**
     * Gets our current position in meters on the field.
     *
     * @return A current position on the field.
     * <p><pi> A translation2d (X and Y on the field) -> {@link #m_kinematics} + A rotation2d (Rot
     * X and Y on the field) -> {@link #nav}
     */
    public Pose2d getPose2d() {
        return m_odometry.getPoseMeters();
    }

    public Pose2d getPose2dEstimator() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Gets the Position of the four SwerveModules.
     *
     * <p>This gets the encoder in the motor (drive) and the encoder on the swerve module.
     */
    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition frontLeftPosition = m_frontLeft.getModulePosition();
        SwerveModulePosition frontRightPosition = m_frontRight.getModulePosition();
        SwerveModulePosition backLeftPosition = m_backLeft.getModulePosition();
        SwerveModulePosition backRightPosition = m_backRight.getModulePosition();
        return new SwerveModulePosition[]{
                frontLeftPosition, frontRightPosition, backLeftPosition, backRightPosition
        };
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot) {

        SmartDashboard.putNumber(getName() + "/Command/X Speed", xSpeed);
        SmartDashboard.putNumber(getName() + "/Command/Y Speed", ySpeed);
        SmartDashboard.putNumber(getName() + "/Command/Rot Speed", rot);
        SmartDashboard.putBoolean(getName() + "/Command/RobotRelative", m_FieldRelativeEnable);

        var alliance = DriverStation.getAlliance();
        Rotation2d robotRotation;

        // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
        //   robotRotation = new Rotation2d(navx.getRotation2d().getRadians() + Rotation2d.fromDegrees(180).getRadians());
        // } else {
        robotRotation = new Rotation2d(navx.getRotation2d().getRadians());
        // }

        // SmartDashboard.putNumber ( "inputRotiation", robotRotation.getDegrees());
        DesiredStates =
                m_kinematics.toSwerveModuleStates(
                        m_FieldRelativeEnable
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, robotRotation)
                                : new ChassisSpeeds(xSpeed, ySpeed, rot));

        if (!m_WheelLock) {
            setModuleStates(DesiredStates);
        } else {
            WheelLock();
        }
    }

    /**
     * Tells our modules what speed to go to
     */
    public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.kDriveMaxSpeed);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Tells our wheels to go to the Wheel Locking position (0 m/s, forming an X)
     */
    public void WheelLock() {
        m_backLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
        m_frontLeft.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
        m_backRight.setDesiredState(new SwerveModuleState(0, new Rotation2d((Math.PI / 4))));
        m_frontRight.setDesiredState(new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4))));
    }

    @Override
    public void periodic() {
        updateOdometry();
        updatePoseEstimatorOdometry();
        super.periodic();
        field.setRobotPose(getPose2dEstimator());
        DesiredStatePublisher.set(DesiredStates);
        CurrentStatePublisher.set(getSwerveModuleStates());
        CurrentSpeedsPublisher.set(getChassisSpeeds());
        CurrentPosePublisher.set(getPose2d());
        CurrentRotPublisher.set(getPose2d().getRotation());
        CurrentPoseEstimatorPublisher.set(getPose2dEstimator());
        GoalPosePublisher.set(goalPose);
    }

    /**
     * Runnable Command.
     *
     * <p>Tells the Wheels when to stop or not based off of a boolean varible named {@link
     * #m_WheelLock}.
     *
     * <p>Used in drive Method
     */
    public Command WheelLockCommand() {

        return this.runOnce(
                () -> {

                    // one-time action goes here
                    // WP - Add code here to toggle the gripper solenoid
                    if (m_WheelLock == true) {
                        m_WheelLock = false;
                    } else if (m_WheelLock == false) {
                        m_WheelLock = true;
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
                    navx.reset();
                });
    }

    public Command SetGyroAdjustmentAngle() {
        // Inline construction of command goes here.
        // Subsystem::RunOnce implicitly requires `this` subsystem.

        return this.runOnce(
                () -> {
                    navx.setAngleAdjustment(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-frl").pose.getRotation().getDegrees());
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
        SwerveModuleState[] targetStates = m_kinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(targetStates);
    }


    /**
     * Resets the Position of the Odometer, given our Current position.
     *
     * @param Pose2d (pose2d) - The current position of the robot on the field. This is a {@link
     *               #resetOdometry(Pose2d)}
     */
    public void resetPose(Pose2d pose2d) {
        resetOdometry(pose2d);
    }

    public void resetPoseEstimator(Pose2d pose2d) {
        m_poseEstimator.resetPosition(navx.getRotation2d(), getSwerveModulePositions(), pose2d);
    }

    /**
     * Reset's the Robots Odometry using the Gyro's Current Rotational Position
     *
     * @param pose2d
     */
    public void resetOdometry(Pose2d pose2d) {
        m_odometry.resetPosition(navx.getRotation2d(), getSwerveModulePositions(), pose2d);
    }

    /**
     * Converts raw module states into chassis speeds
     *
     * @return chassisSpeeds --> A reading of the speed in m/s our robot is going.
     */
    public ChassisSpeeds getChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public Command PathFindLeft() {
        return this.defer(() -> {
            Pose2d goalLeft;
            if (DriverStation.getAlliance().orElseThrow().equals(Alliance.Red)) {
                goalLeft = getPose2dEstimator().nearest(Constants.Poses.PositionsLeftRed);
                SmartDashboard.putNumber("goalLeft/X", goalLeft.getX());
                SmartDashboard.putNumber("goalLeft/Y", goalLeft.getY());
                SmartDashboard.putNumber("goalLeft/Rot", goalLeft.getRotation().getDegrees());
                return AutoBuilder.pathfindToPose(goalLeft, RobotContainer.SPEED_CONSTRAINTS);


            } else {
                goalLeft = getPose2dEstimator().nearest(Constants.Poses.PositionsLeftBlue);
                SmartDashboard.putNumber("goalLeft/X", goalLeft.getX());
                SmartDashboard.putNumber("goalLeft/Y", goalLeft.getY());
                SmartDashboard.putNumber("goalLeft/Rot", goalLeft.getRotation().getDegrees());
                return AutoBuilder.pathfindToPose(goalLeft, RobotContainer.SPEED_CONSTRAINTS);
            }
        });
    }

    public Command PathFindRight() {
        return this.defer(() -> {
            Pose2d goalRight;

            if (DriverStation.getAlliance().orElseThrow().equals(Alliance.Red)) {
                goalRight = getPose2dEstimator().nearest(Constants.Poses.PositionsRightRed);
                SmartDashboard.putNumber("goalRight/X", goalRight.getX());
                SmartDashboard.putNumber("goalRight/Y", goalRight.getY());
                SmartDashboard.putNumber("goalRight/Rot", goalRight.getRotation().getDegrees());
                return AutoBuilder.pathfindToPose(goalRight, RobotContainer.SPEED_CONSTRAINTS);

            } else {
                goalRight = getPose2dEstimator().nearest(Constants.Poses.PositionsRightBlue);
                SmartDashboard.putNumber("goalRight/X", goalRight.getX());
                SmartDashboard.putNumber("goalRight/Y", goalRight.getY());
                SmartDashboard.putNumber("goalRight/Rot", goalRight.getRotation().getDegrees());
                return AutoBuilder.pathfindToPose(goalRight, RobotContainer.SPEED_CONSTRAINTS);
            }
        });
    }


    public Command PathGenerateLeft() {
        return this.defer(() -> {
            Pose2d goalRight = getPose2dEstimator().nearest(Constants.Poses.PositionsRightBlue);
            SmartDashboard.putNumber("goalRight/X", goalRight.getX());
            SmartDashboard.putNumber("goalRight/Y", goalRight.getY());
            SmartDashboard.putNumber("goalRight/Rot", goalRight.getRotation().getDegrees());
            return generatePath(getPose2dEstimator(), goalRight, RobotContainer.SPEED_CONSTRAINTS);
        });
    }


    public Command PathGenerateRight() {
        return this.defer(() -> {
            Pose2d goalRight = getPose2dEstimator().nearest(Constants.Poses.PositionsRightBlue);
            SmartDashboard.putNumber("goalRight/X", goalRight.getX());
            SmartDashboard.putNumber("goalRight/Y", goalRight.getY());
            SmartDashboard.putNumber("goalRight/Rot", goalRight.getRotation().getDegrees());
            return generatePath(getPose2dEstimator(), goalRight, RobotContainer.SPEED_CONSTRAINTS);
        });
    }

    /**
     * Creates Path from current pose to goal pose following path constraints
     * Could be used as a substitue to PathPlanner onTheFly
     *
     * @param currentPose
     * @param goalPose
     * @param pathConstraints
     * @return Autobuilder followPath Command
     */
    public Command generatePath(Pose2d currentPose, Pose2d goalPose, PathConstraints pathConstraints) {
        Translation2d currentPosition = currentPose.getTranslation();
        Translation2d goalPosition = goalPose.getTranslation();
        Rotation2d goalRotation = goalPose.getRotation();

        Translation2d currentToGoalDistance = goalPosition.minus(currentPosition);
        Rotation2d currentToGoalDirection = currentToGoalDistance.getAngle();

        //Flips the goal Pose if the alliance is Red
        if (DriverStation.getAlliance().orElseThrow().equals(Alliance.Red)) {
            goalPosition = FlippingUtil.flipFieldPosition(goalPosition);
            goalRotation = FlippingUtil.flipFieldRotation(goalRotation);
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPosition.getX(), currentPose.getY(), currentToGoalDirection),
                new Pose2d(goalPosition.getX(), goalPosition.getY(), currentToGoalDirection));

        PathPlannerPath path =
                new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0, goalRotation));
        path.preventFlipping = true;

        return AutoBuilder.followPath(path);
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
                m_frontLeft.getModuleState(),
                m_frontRight.getModuleState(),
                m_backLeft.getModuleState(),
                m_backRight.getModuleState()
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
                    if (m_FieldRelativeEnable == true) {
                        m_FieldRelativeEnable = false;
                        // System.out.println("I am Here 2");
                    } else if (m_FieldRelativeEnable == false) {
                        m_FieldRelativeEnable = true;
                        // System.out.println("I am Here 3");
                    }
                });
    }

    public void SetFieldRelativeEnable(boolean fieldRelative) {
        m_FieldRelativeEnable = fieldRelative;
    }

    /**
     * Updates our current Odometry
     */
    public void updateOdometry() {
        m_odometry.update(navx.getRotation2d(), getSwerveModulePositions());
    }

    public Command resetPoseEstimatorCmd() {
        return this.runOnce(() -> resetPoseEstimator(getPose2dEstimator()));
    }

    public void updatePoseEstimatorOdometry() {
        m_poseEstimator.update(
                navx.getRotation2d(),
                new SwerveModulePosition[]{
                        m_frontLeft.getModulePosition(),
                        m_frontRight.getModulePosition(),
                        m_backLeft.getModulePosition(),
                        m_backRight.getModulePosition()
                });


        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        if (useMegaTag2 == false) {
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
                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt1.pose,
                        mt1.timestampSeconds);
            }
        } else if (useMegaTag2 == true) {
            LimelightHelpers.SetRobotOrientation("limelight-frl", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), navx.getRate(), navx.getPitch(), 0, navx.getRoll(), 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-frl");
            if (Math.abs(navx.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            {
                doRejectUpdate = true;
            }
            if (mt2 == null) {
                doRejectUpdate = true;
            } else if (mt2.tagCount == 0) {
                doRejectUpdate = true;
            }
            if (!doRejectUpdate) {

                m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                m_poseEstimator.addVisionMeasurement(
                        mt2.pose,
                        mt2.timestampSeconds);
            }
        }
    }

    /**
     * Stops all the motors on the SwerveModules
     */
    public void stopModules() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Runnable Command. Runs the {@link #stopModules()} Command.
     */
    public Command Break() {
        return this.run(
                () -> {
                    stopModules();
                });
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
                "Odometry/navx/Orientation", () -> navx.getRotation2d().getDegrees(), null);
        builder.addBooleanProperty(
                "FieldRelativeEnabled",
                () -> this.m_FieldRelativeEnable,
                (boolean fre) -> m_FieldRelativeEnable = fre);
        builder.addDoubleProperty("EstimatedOdometry/Pose/X", () -> getPose2dEstimator().getX(), null);
        builder.addDoubleProperty("EstimatedOdometry/Pose/Y", () -> getPose2dEstimator().getY(), null);
        builder.addDoubleProperty(
                "EstimatedOdometry/Pose/Rot", () -> getPose2dEstimator().getRotation().getDegrees(), null);
        builder.addDoubleProperty("GOALPOSE/X", () -> getGoal().getX(), null);
        builder.addDoubleProperty("GOALPOSE/Y", () -> getGoal().getY(), null);
        builder.addDoubleProperty("GOALPOSE/ROT", () -> getGoal().getRotation().getRadians(), null);
        SmartDashboard.putData("DriveTrain/" + m_frontLeft.getName(), m_frontLeft);
        SmartDashboard.putData("DriveTrain/" + m_frontRight.getName(), m_frontRight);
        SmartDashboard.putData("DriveTrain/" + m_backLeft.getName(), m_backLeft);
        SmartDashboard.putData("DriveTrain/" + m_backRight.getName(), m_backRight);
        SmartDashboard.putData("field", field);
        builder.addDoubleProperty("GYRO ANGLE", () -> navx.getAngle(), null);
        SmartDashboard.putData("NAVX DATA", navx);
        builder.addDoubleProperty("NAVX ROTATION", () -> navx.getRotation2d().getDegrees(), null);
        builder.addDoubleProperty("NAVX AngleAdjustment", () -> navx.getAngleAdjustment(), null);
    }
}
