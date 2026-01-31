// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static edu.wpi.first.math.MathUtil.clamp;
import static edu.wpi.first.math.kinematics.SwerveModuleState.optimize;
import static java.lang.Math.min;
import static java.util.Objects.requireNonNull;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
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
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import frc.robot.support.limelight.LimelightHelpers;
import java.util.Optional;

/**
 * Represents a swerve drive style drivetrain. In here, we initialize our swerve modules (example ->
 * {@link #flSwerve}), Get input from autonomous and initialize our odometry ->
 * {@link #swerveDriveOdometry}. Various other DriveTrain Related thing are initialized here too.
 */
public class Drivetrain extends SubsystemBase {

    private static final double NOMINAL_BATT_VOLTS = 12.0;

    private final double SIM_VOLTS_PER_RADIAN_TURN_VOLTAGE = 48.0;

    private final DrivetrainContext context;

    // Scale from desired wheel speed (meters-per-second) to motor voltage (V)
    private final double simVoltsMetPerSec;

    private final Field2d field = new Field2d();

    private final SwerveDriveKinematics swerveDriveKinematics;

    // The gyro object. Gyro gives the robots rotation/ where the robot is pointed.
    private final AHRS navXSensorModule;

    /**
     * Gets the current gyro rotation
     */
    private Rotation2d getGyroRotation() {
        return navXSensorModule.getRotation2d();
    }

    public final SwerveModule flSwerve;

    public final SwerveModule frSwerve;

    public final SwerveModule rlSwerve;

    public final SwerveModule rrSwerve;

    private final SwerveModuleSim flSwerveSim;

    private final SwerveModuleSim frSwerveSim;

    private final SwerveModuleSim rlSwerveSim;

    private final SwerveModuleSim rrSwerveSim;

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

    private boolean fieldRelativeEnable = false;

    private Pose2d goalPose;

    private SwerveModuleState[] desiredStates;

    // Simulated gyro yaw (only used in simulation)
    private Rotation2d simYaw = new Rotation2d();

    private double lastSimTime = Timer.getFPGATimestamp();

    /**
     * Instantiates a new Drivetrain with default {@link DrivetrainContext}
     */
    public Drivetrain() {
        this(DrivetrainContext.defaults());
    }

    /**
     * Instantiates a new Drivetrain subsystem with the specified settings
     *
     * @param context The DrivetrainSettings to apply to this instance
     */
    public Drivetrain(final DrivetrainContext context) {
        requireNonNull(context, "DrivetrainContext cannot be null");

        this.context = context;

        this.simVoltsMetPerSec = NOMINAL_BATT_VOLTS / this.getMaxSpeed();

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

        // Initialize NavX gyro
        this.navXSensorModule = new AHRS(AHRS.NavXComType.kMXP_SPI);

        this.context
                .getRobotConfig()
                .ifPresentOrElse(
                        robotConfig -> {
                            AutoBuilder.configure(
                                    this::getPose2dEstimator, // Robot pose supplier (uses vision-fused pose)
                                    this::resetOdometry, // Method to reset odometry (will be called if your auto
                                    // has a starting pose)
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
                            // NOTE: PathPlanner AutoBuilder not configured - auto routines will not be available
                            System.out.println("WARNING: Unable to obtain RobotConfig during Drivetrain creation. "
                                    + "PathPlanner AutoBuilder not configured. Auto routines will not be available.");
                        });

        this.swerveDriveKinematics = new SwerveDriveKinematics(
                Constants.Drive.SMFrontLeftLocation,
                Constants.Drive.SMFrontRightLocation,
                Constants.Drive.SMBackLeftLocation,
                Constants.Drive.SMBackRightLocation);

        this.frSwerve = new SwerveModule(this.context.getFrSwerveContext());
        this.flSwerve = new SwerveModule(this.context.getFlSwerveContext());
        this.rlSwerve = new SwerveModule(this.context.getRlSwerveContext());
        this.rrSwerve = new SwerveModule(this.context.getRrSwerveContext());

        this.frSwerveSim = new SwerveModuleSim(context.getFrSwerveContext());
        this.flSwerveSim = new SwerveModuleSim(context.getFlSwerveContext());
        this.rlSwerveSim = new SwerveModuleSim(context.getRlSwerveContext());
        this.rrSwerveSim = new SwerveModuleSim(context.getRrSwerveContext());

        // initializes odometry
        this.swerveDriveOdometry = new SwerveDriveOdometry(
                this.swerveDriveKinematics, this.getGyroRotation(), this.getSwerveModulePositions());

        this.swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
                this.swerveDriveKinematics,
                this.getGyroRotation(),
                this.getSwerveModulePositions(),
                new Pose2d(),
                this.context.getStateStdDevs(),
                this.context.getVisionMeasurementStdDevs());

        this.addChild(flSwerve.getName(), flSwerve);
        this.addChild(frSwerve.getName(), frSwerve);
        this.addChild(rlSwerve.getName(), rlSwerve);
        this.addChild(rrSwerve.getName(), rrSwerve);
        this.addChild("navx", this.navXSensorModule);

        // Register for automatic telemetry capture
        Telemetry.registerSubsystem("Drivetrain", this::captureTelemetry);
    }

    /**
     * Captures telemetry data for this subsystem. Called automatically by Telemetry.periodic().
     * Data is organized by telemetry level for efficient capture at different verbosity settings.
     *
     * @param prefix The telemetry key prefix (typically "Drivetrain")
     */
    private void captureTelemetry(String prefix) {
        // MATCH level - essential data for competition analysis
        Telemetry.record(prefix + "/Pose", this.getPose2dEstimator(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Speeds", this.getChassisSpeeds(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/Heading", this.getHeading(), TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/FieldRelative", this.fieldRelativeEnable, TelemetryLevel.MATCH);
        Telemetry.record(prefix + "/WheelLock", this.wheelLock, TelemetryLevel.MATCH);

        // LAB level - detailed data for practice and tuning
        Telemetry.record(prefix + "/ModuleStates/Current", this.getSwerveModuleStates(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/ModuleStates/Desired", this.desiredStates, TelemetryLevel.LAB);
        Telemetry.record(prefix + "/OdometryPose", this.getPose2d(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/GyroAngle", this.navXSensorModule.getAngle(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/GyroPitch", this.navXSensorModule.getPitch(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/GyroRoll", this.navXSensorModule.getRoll(), TelemetryLevel.LAB);
        Telemetry.record(prefix + "/GyroRate", this.navXSensorModule.getRate(), TelemetryLevel.LAB);

        if (this.goalPose != null) {
            Telemetry.record(prefix + "/GoalPose", this.goalPose, TelemetryLevel.LAB);
        }

        // VERBOSE level - maximum detail for deep debugging
        Telemetry.record(prefix + "/NavX/Connected", this.navXSensorModule.isConnected(), TelemetryLevel.VERBOSE);
        Telemetry.record(prefix + "/NavX/Calibrating", this.navXSensorModule.isCalibrating(), TelemetryLevel.VERBOSE);
        Telemetry.record(
                prefix + "/NavX/AngleAdjustment", this.navXSensorModule.getAngleAdjustment(), TelemetryLevel.VERBOSE);
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
     * <p>
     * <pi> A translation2d (X and Y on the field) -> {@link #swerveDriveKinematics} + A rotation2d (Rot X and Y
     * on the field) -> {@link #navXSensorModule}
     */
    private Pose2d getPose2d() {
        return this.swerveDriveOdometry.getPoseMeters();
    }

    public Pose2d getPose2dEstimator() {
        return this.swerveDrivePoseEstimator.getEstimatedPosition();
    }

    /**
     * Adds a vision measurement to the pose estimator.
     * Called by VisionSubsystem when new vision estimates are available.
     *
     * @param visionPose The pose estimate from vision
     * @param timestampSeconds The timestamp of the vision measurement
     * @param stdDevs The standard deviations (x, y, theta) for this measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {

        this.swerveDrivePoseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);

        SmartDashboard.putNumber("Vision/AcceptedPose/X", visionPose.getX());
        SmartDashboard.putNumber("Vision/AcceptedPose/Y", visionPose.getY());
        SmartDashboard.putNumber("Vision/AcceptedPose/Timestamp", timestampSeconds);
    }

    /**
     * Tells our modules what speed to go to
     */
    private void setModuleStates(SwerveModuleState[] swerveModuleStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveModule.DRIVE_MAX_SPEED);
        this.flSwerve.setDesiredState(swerveModuleStates[0]);
        this.frSwerve.setDesiredState(swerveModuleStates[1]);
        this.rlSwerve.setDesiredState(swerveModuleStates[2]);
        this.rrSwerve.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Tells our wheels to go to the Wheel Locking position (0 m/s, forming an X)
     */
    private void lockWheels() {
        this.rlSwerve.setDesiredState(this.context.getFullStopAt135Degrees());
        this.flSwerve.setDesiredState(this.context.getFullStopAt45Degrees());
        this.rrSwerve.setDesiredState(this.context.getFullStopAt45Degrees());
        this.frSwerve.setDesiredState(this.context.getFullStopAt135Degrees());
    }

    /**
     * Gets the Position of the four SwerveModules.
     *
     * <p>
     * This gets the encoder in the motor (drive) and the encoder on the swerve module.
     */
    private SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            this.flSwerve.getModulePosition(),
            this.frSwerve.getModulePosition(),
            this.rlSwerve.getModulePosition(),
            this.rrSwerve.getModulePosition()
        };
    }

    private void driveRobotRelative(final ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        this.desiredStates = this.swerveDriveKinematics.toSwerveModuleStates(targetSpeeds);
        this.setModuleStates(this.desiredStates);
    }

    /**
     * Updates our current Odometry
     */
    private void updateOdometry() {
        if (RobotBase.isSimulation()) return;
        this.swerveDriveOdometry.update(this.getGyroRotation(), this.getSwerveModulePositions());
    }

    /**
     * Updates odometry using simulated module positions.
     * Called from simulationPeriodic() after physics update.
     */
    private void updateOdometrySim(Rotation2d heading, SwerveModulePosition[] positions) {
        this.swerveDriveOdometry.update(heading, positions);
    }

    private void updatePoseEstimatorOdometry() {
        if (RobotBase.isSimulation()) return;
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
                    0,
                    navXSensorModule.getPitch(),
                    0,
                    navXSensorModule.getRoll(),
                    0);

            // NOTE: This is where we had issues in 2025 because mt2 required that we had correct robot orientation
            // relative to the field. We couldn't figure out how to adjust our Gyro at the beginning of the match
            // based on vision. So what we tried to do, was manually place the robot on the field at the beginning
            // of the match. And if that of 2-3 degrees/rads. then that would blow auto up.
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
        this.swerveDriveOdometry.resetPosition(this.getGyroRotation(), this.getSwerveModulePositions(), pose2d);

        if (RobotBase.isSimulation()) {
            // Reset simulated module positions to keep odometry consistent
            this.flSwerveSim.resetPosition();
            this.frSwerveSim.resetPosition();
            this.rlSwerveSim.resetPosition();
            this.rrSwerveSim.resetPosition();

            // Keep simulated gyro aligned with newPose
            this.simYaw = pose2d.getRotation();

            // Reset odometry with the simulated positions (now zeroed)
            SwerveModulePosition[] simPositions = new SwerveModulePosition[] {
                this.flSwerveSim.getPosition(),
                this.frSwerveSim.getPosition(),
                this.rlSwerveSim.getPosition(),
                this.rrSwerveSim.getPosition()
            };

            this.swerveDriveOdometry.resetPosition(this.simYaw, simPositions, pose2d);
            this.swerveDrivePoseEstimator.resetPosition(this.simYaw, simPositions, pose2d);
        } else {
            this.swerveDriveOdometry.resetPosition(this.getGyroRotation(), this.getSwerveModulePositions(), pose2d);
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
            return this.getGyroRotation();
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
            this.flSwerve.getModuleState(),
            this.frSwerve.getModuleState(),
            this.rlSwerve.getModuleState(),
            this.rrSwerve.getModuleState()
        };
    }

    private Pose2d refreshGoalPose2d() {
        // NOTE: This is hard-coded to red, because the field is 0'd to the blue wall. So our goal
        // is naturally opposite blue.
        this.goalPose = this.getPose2dEstimator().nearest(Constants.Poses.PositionsRed);
        return this.goalPose;
    }

    /**
     * Stops all the motors on the SwerveModules
     */
    public void stopModules() {
        this.flSwerve.stopMotors();
        this.frSwerve.stopMotors();
        this.rlSwerve.stopMotors();
        this.rrSwerve.stopMotors();
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot    Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        // Commanded inputs (m/s, rad/s)
        SmartDashboard.putNumber(getName() + "/Cmd/xSpeed_in", xSpeed);
        SmartDashboard.putNumber(getName() + "/Cmd/ySpeed_in", ySpeed);
        SmartDashboard.putNumber(getName() + "/Cmd/rot_in", rot);
        SmartDashboard.putBoolean(getName() + "/Cmd/FieldRelative", this.fieldRelativeEnable);

        Rotation2d rawHeading = getHeading();

        ChassisSpeeds speeds = fieldRelativeEnable
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, rawHeading)
                : new ChassisSpeeds(xSpeed, ySpeed, rot);

        // IMPORTANT: NavX is CW+, WPILib is CCW+. Negate heading for fromFieldRelativeSpeeds.
        SmartDashboard.putNumber(getName() + "/HeadingUsedDeg", rawHeading.getDegrees());

        // What we will pass to kinematics
        SmartDashboard.putNumber(getName() + "/Chassis/vx", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber(getName() + "/Chassis/vy", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber(getName() + "/Chassis/omega", speeds.omegaRadiansPerSecond);

        // Compute and apply
        this.desiredStates = this.swerveDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(this.desiredStates, SwerveModule.DRIVE_MAX_SPEED);

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

        // Log alliance for debugging path flipping
        Optional<Alliance> alliance = DriverStation.getAlliance();
        SmartDashboard.putString("Auto/Alliance", alliance.map(Enum::name).orElse("NOT SET"));
    }

    @Override
    public void simulationPeriodic() {
        // 1. Compute elapsed time since last loop
        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - this.lastSimTime;
        this.lastSimTime = currentTime;

        // 2. Skip if no desired states yet (e.g., before first drive command)
        if (this.desiredStates == null) {
            return;
        }

        // 3. Compute each module’s commanded (optimized) state
        SwerveModuleState frontLeftOptimized = optimize(this.desiredStates[0], this.flSwerveSim.getTurnAngle());
        SwerveModuleState frontRightOptimized = optimize(this.desiredStates[1], this.frSwerveSim.getTurnAngle());
        SwerveModuleState rearLeftOptimized = optimize(this.desiredStates[2], this.rlSwerveSim.getTurnAngle());
        SwerveModuleState rearRightOptimized = optimize(this.desiredStates[3], this.rrSwerveSim.getTurnAngle());

        // 4. Apply drive voltages (scale m/s → ±12 V)
        this.flSwerveSim.setDriveVoltage(Math.copySign(
                min(Math.abs(frontLeftOptimized.speedMetersPerSecond) * this.simVoltsMetPerSec, NOMINAL_BATT_VOLTS),
                frontLeftOptimized.speedMetersPerSecond));
        this.frSwerveSim.setDriveVoltage(Math.copySign(
                min(Math.abs(frontRightOptimized.speedMetersPerSecond) * this.simVoltsMetPerSec, NOMINAL_BATT_VOLTS),
                frontRightOptimized.speedMetersPerSecond));
        this.rlSwerveSim.setDriveVoltage(Math.copySign(
                min(Math.abs(rearLeftOptimized.speedMetersPerSecond) * this.simVoltsMetPerSec, NOMINAL_BATT_VOLTS),
                rearLeftOptimized.speedMetersPerSecond));
        this.rrSwerveSim.setDriveVoltage(Math.copySign(
                min(Math.abs(rearRightOptimized.speedMetersPerSecond) * this.simVoltsMetPerSec, NOMINAL_BATT_VOLTS),
                rearRightOptimized.speedMetersPerSecond));

        // 5. Apply turn voltages (simple proportional control on angle error)
        double frontLeftError =
                frontLeftOptimized.angle.minus(flSwerveSim.getTurnAngle()).getRadians();
        double frontRightError =
                frontRightOptimized.angle.minus(frSwerveSim.getTurnAngle()).getRadians();
        double rearLeftError =
                rearLeftOptimized.angle.minus(rlSwerveSim.getTurnAngle()).getRadians();
        double rearRightError =
                rearRightOptimized.angle.minus(rrSwerveSim.getTurnAngle()).getRadians();

        this.flSwerveSim.setTurnVoltage(
                clamp(SIM_VOLTS_PER_RADIAN_TURN_VOLTAGE * frontLeftError, -NOMINAL_BATT_VOLTS, NOMINAL_BATT_VOLTS));
        this.frSwerveSim.setTurnVoltage(
                clamp(SIM_VOLTS_PER_RADIAN_TURN_VOLTAGE * frontRightError, -NOMINAL_BATT_VOLTS, NOMINAL_BATT_VOLTS));
        this.rlSwerveSim.setTurnVoltage(
                clamp(SIM_VOLTS_PER_RADIAN_TURN_VOLTAGE * rearLeftError, -NOMINAL_BATT_VOLTS, NOMINAL_BATT_VOLTS));
        this.rrSwerveSim.setTurnVoltage(
                clamp(SIM_VOLTS_PER_RADIAN_TURN_VOLTAGE * rearRightError, -NOMINAL_BATT_VOLTS, NOMINAL_BATT_VOLTS));

        flSwerveSim.update(dt);
        frSwerveSim.update(dt);
        rlSwerveSim.update(dt);
        rrSwerveSim.update(dt);

        // 7. Build module states for kinematics
        SwerveModuleState[] states = new SwerveModuleState[] {
            new SwerveModuleState(this.flSwerveSim.getWheelSpeedMetersPerSecond(), this.flSwerveSim.getTurnAngle()),
            new SwerveModuleState(this.frSwerveSim.getWheelSpeedMetersPerSecond(), this.frSwerveSim.getTurnAngle()),
            new SwerveModuleState(this.rlSwerveSim.getWheelSpeedMetersPerSecond(), this.rlSwerveSim.getTurnAngle()),
            new SwerveModuleState(this.rrSwerveSim.getWheelSpeedMetersPerSecond(), this.rrSwerveSim.getTurnAngle())
        };

        // 8. Convert to chassis speeds and integrate heading
        ChassisSpeeds chassisSpeeds = this.swerveDriveKinematics.toChassisSpeeds(states);
        this.simYaw = this.simYaw.plus(Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * dt));

        // 9. Update pose estimator and odometry with sim yaw and module positions
        SwerveModulePosition[] simPositions = new SwerveModulePosition[] {
            this.flSwerveSim.getPosition(),
            this.frSwerveSim.getPosition(),
            this.rlSwerveSim.getPosition(),
            this.rrSwerveSim.getPosition()
        };

        this.swerveDrivePoseEstimator.update(this.simYaw, simPositions);
        this.updateOdometrySim(this.simYaw, simPositions);

        // 10. Push pose to Field2d for visualization
        this.field.setRobotPose(this.swerveDrivePoseEstimator.getEstimatedPosition());
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
                "Odometry/navx/Orientation", () -> this.getGyroRotation().getDegrees(), null);
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

        SmartDashboard.putData("DriveTrain/" + this.flSwerve.getName(), this.flSwerve);
        SmartDashboard.putData("DriveTrain/" + this.frSwerve.getName(), this.frSwerve);
        SmartDashboard.putData("DriveTrain/" + this.rlSwerve.getName(), this.rlSwerve);
        SmartDashboard.putData("DriveTrain/" + this.rrSwerve.getName(), this.rrSwerve);
        SmartDashboard.putData("field", this.field);

        builder.addDoubleProperty("GYRO ANGLE", this.navXSensorModule::getAngle, null);
        SmartDashboard.putData("NAVX DATA", this.navXSensorModule);
        builder.addDoubleProperty("GYRO ROTATION", () -> this.getGyroRotation().getDegrees(), null);
        builder.addDoubleProperty("NAVX AngleAdjustment", this.navXSensorModule::getAngleAdjustment, null);
    }
}
