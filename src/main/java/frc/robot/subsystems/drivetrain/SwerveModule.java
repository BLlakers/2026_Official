// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import static java.util.Objects.requireNonNull;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import frc.robot.support.sparkmax.TeamSpark;

/**
 * This is the code to run a single swerve module. SwerveModules have a turning motor, a drive motor, and associated
 * turning and drive encoders. It is called by the Drivetrain subsystem
 */
public class SwerveModule extends SubsystemBase {

    private static final double TOTAL_ROTATIONAL_RANGE = 2 * Math.PI;

    private static final double POSITION_CONVERSION_FACTOR =
            (Constants.Conversion.kWheelDiameterM * Math.PI) / Constants.Conversion.DriveGearRatio;

    private static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR / 60;

    // Below this speed (m/s), we skip optimize() and stop both motors.
    // Prevents turn motor shimmy caused by optimize() freely flipping the
    // desired angle by PI when negating zero speed has no effect.
    private static final double DESIRED_SPEED_DEADBAND = 0.01;

    private static final int TURNING_MOTOR_ASSUMED_FREQUENCY = 242;

    private final SwerveModuleContext context;

    private final String telemetryPrefix;

    private final TeamSpark driveMotor;

    private final TeamSpark turningMotor;

    private final DutyCycleEncoder turningMotorEncoder;

    // Retain our last desired state to support simulation
    private SwerveModuleState lastDesiredState = new SwerveModuleState();

    private double lastDrivePercent = 0.0;
    private double lastTurnPercent = 0.0;

    // Tracks the last optimized angle so that optimize() makes consistent flip decisions.
    // Without this, optimize() compares fresh kinematics angles against the noisy current
    // encoder reading each cycle, causing flip-flop oscillation at the 90° decision boundary.
    private Rotation2d lastAngle = new Rotation2d();

    /**
     * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
     */
    public SwerveModule(final SwerveModuleContext context) {
        requireNonNull(context, "SwerveModuleContext cannot be null");

        this.context = context;
        this.telemetryPrefix = "Drivetrain/" + context.getName();

        this.setName(this.context.getName());

        this.driveMotor = this.context.getDriveMotor();

        // PWM encoder from CTRE mag encoders
        this.turningMotor = this.context.getTurningMotor();
        this.turningMotor.configure(
                assembleTurnMotorConfig(), ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        this.turningMotorEncoder = new DutyCycleEncoder(
                this.context.getTurnEncoderPWMChannel(), TOTAL_ROTATIONAL_RANGE, this.context.getTurnOffset());

        this.turningMotorEncoder.setAssumedFrequency(TURNING_MOTOR_ASSUMED_FREQUENCY);

        this.driveMotor.configure(
                this.assembleDriveMotorConfig(this.context.isInverted()),
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    /**
     * Prepares and configures a {@link SparkFlexConfig} to be applied to this swerve module's NEO Vortex drive motor
     *
     * @return The SparkFlexConfig
     */
    private SparkFlexConfig assembleDriveMotorConfig(boolean inverted) {
        SparkFlexConfig config = new SparkFlexConfig();
        config.inverted(inverted).idleMode(IdleMode.kBrake);
        config.encoder
                .positionConversionFactor(POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        PIDSettings pidSettings = this.context.getDriveMotorPIDSettings();
        config.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(pidSettings.p(), pidSettings.i(), pidSettings.d());

        // NOTE: Added per FRC (Sophia recommendation)
        config.smartCurrentLimit(this.context.getDriveMotorStallCurrentLimit());
        return config;
    }

    private SparkMaxConfig assembleTurnMotorConfig() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(this.context.getTurnMotorCurrentLimit());
        config.idleMode(IdleMode.kBrake); // TODO: Consider kBrake... should use coast instead?
        return config;
    }

    /**
     * Returns the current state of the module. <pi> This takes a current velocity for each different drive encoder and
     * a current angle.
     *
     * @return The current state of each Swerve Module. --> The speed and angle of a Module
     */
    public SwerveModuleState getModuleState() {
        // the getVelocity() function normally returns RPM but is scaled in the
        // SwerveModule constructor to return actual wheel speed

        return new SwerveModuleState(
                this.driveMotor.getEncoder().getVelocity(), Rotation2d.fromRadians(this.turningMotorEncoder.get()));
    }

    /**
     * Sets the desired state for the module. This implementation will also retain the desired state internally in
     * support for simulation. See {@link SwerveModule#getLastDesiredState}
     *
     * <p>
     * This means the speed it should be going and the angle it should be going.
     *
     * @param desiredState
     *            Desired state with speed and angle.
     */
    public void setDesiredState(final SwerveModuleState desiredState) {

        // When speed is near-zero, hold current wheel angle and stop both motors.
        // This prevents optimize() from freely flipping the angle by PI
        // (since negating zero speed is still zero), which causes turn motor shimmy.
        if (Math.abs(desiredState.speedMetersPerSecond) < DESIRED_SPEED_DEADBAND) {
            this.driveMotor.set(0);
            this.turningMotor.set(0);
            this.lastDesiredState = desiredState;
            return;
        }

        // Read encoder once for consistency within this cycle.
        // Previously the encoder was read separately for optimize() and for the angle error
        // calculation, which could produce different values near wraparound boundaries.
        Rotation2d currentAngle = getModulePosition().angle;

        double unoptimizedDesiredAngle = desiredState.angle.getRadians();

        // Optimize against the last committed angle (not the current encoder reading).
        // This prevents flip-flop oscillation when the wheel sits near the 90° decision
        // boundary of optimize(). Once a representation is chosen, it stays consistent.
        desiredState.optimize(this.lastAngle);
        this.lastAngle = desiredState.angle;

        // Simple proportional turn control: error / 2π normalizes to [-0.5, 0.5], then
        // gain of 3 scales to motor output (saturates at ±1.5 before clamp).
        // Output is clamped to ±60% — see NEO_MOTOR_SMOKE_EVENT.md for rationale.
        // MathUtil.angleModulus handles wraparound correctly (always shortest path).
        double angleError = MathUtil.angleModulus(desiredState.angle.getRadians() - currentAngle.getRadians());
        double turnOutput = MathUtil.clamp((angleError / TOTAL_ROTATIONAL_RANGE) * 2, -0.6, 0.6);

        this.turningMotor.set(turnOutput);

        double driveMotorPercentPower = desiredState.speedMetersPerSecond / this.context.getDriveMotorMaxSpeed();
        this.driveMotor.set(driveMotorPercentPower);

        this.publishTelemetry(
                driveMotorPercentPower,
                turnOutput,
                angleError,
                unoptimizedDesiredAngle,
                desiredState.angle.getRadians(),
                currentAngle.getRadians());

        this.lastDesiredState = desiredState;

        this.lastDrivePercent = driveMotorPercentPower;
        this.lastTurnPercent = turnOutput;
    }

    public double getLastDrivePercent() {
        return lastDrivePercent;
    }

    public double getLastTurnPercent() {
        return lastTurnPercent;
    }

    /**
     * This gets a current Position (Distance per rotation in meters) for each different drive encoder and a current
     * angle from the Duty Cycle encoder.
     *
     * @return The current Position of each Swerve Module
     */
    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
                this.driveMotor.getEncoder().getPosition(), Rotation2d.fromRadians(this.turningMotorEncoder.get()));
    }

    /**
     * Obtains the last desired state that has been assigned
     *
     * @return SwerveModuleState The last desired state
     */
    public SwerveModuleState getLastDesiredState() {
        return lastDesiredState;
    }

    /**
     * Publishes telemetry values to NetworkTables for live dashboard visibility.
     */
    private void publishTelemetry(
            double driveMotorPercentPower,
            double turnMotorPercentPower,
            double signedAngleDifference,
            double unoptimizedDesiredAngle,
            double desiredAngle,
            double currentAngle) {

        Telemetry.publish(telemetryPrefix + "/Drive/OutputPercent", driveMotorPercentPower, TelemetryLevel.MATCH);
        Telemetry.publish(telemetryPrefix + "/Turn/OutputPercent", turnMotorPercentPower, TelemetryLevel.MATCH);
        Telemetry.publish(telemetryPrefix + "/Turn/AngleError", signedAngleDifference, TelemetryLevel.MATCH);
        Telemetry.publish(telemetryPrefix + "/Turn/(U)DesiredAngle", unoptimizedDesiredAngle, TelemetryLevel.MATCH);
        Telemetry.publish(telemetryPrefix + "/Turn/DesiredAngle", desiredAngle, TelemetryLevel.MATCH);
        Telemetry.publish(telemetryPrefix + "/Turn/CurrentAngle", currentAngle, TelemetryLevel.MATCH);
    }

    /**
     * Tells the drive and turning motor to stop
     */
    public void stopMotors() {
        this.driveMotor.set(0);
        this.turningMotor.set(0);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.publishConstInteger("TurnMotor/ID", this.turningMotor.getDeviceId());
        builder.publishConstInteger("DriveMotor/ID", this.driveMotor.getDeviceId());
        builder.addDoubleProperty(
                "TurnMotor/Angle", () -> Units.radiansToDegrees(this.turningMotorEncoder.get()), null);
        builder.addDoubleProperty("DriveMotor/Pos", this.driveMotor::getPosition, null);
        builder.addDoubleProperty("DriveMotor/Vel", this.driveMotor::getVelocity, null);
        builder.addDoubleProperty("TurnMotor/Encoder/AbsolutePosition", this.turningMotorEncoder::get, null);
        builder.addDoubleProperty(
                "TurnMotor/Encoder/TurningEncoderPosition", this.turningMotor.getEncoder()::getPosition, null);
        builder.setSafeState(this::stopMotors);
    }
}
