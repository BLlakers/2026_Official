package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Robot;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSpark;
import frc.robot.support.sparkmax.TeamSparkFlexImpl;
import frc.robot.support.sparkmax.TeamSparkFlexSimImpl;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Getter;

@Builder
public class SwerveModuleContext {

    public static SwerveModuleContext defaults() {
        return SwerveModuleContext.builder().build();
    }

    // Drive motor CAN ID
    private final int driveMotorId;

    // Turning motor CAN ID
    private final int turningMotorId;

    // Name of the SwerveModule
    @Getter
    private String name;

    // DIO input for the drive encoder channel B
    @Getter
    private final int turnEncoderPWMChannel;

    // Offset from 0 to 1 for the home position of the encoder
    @Getter
    private final double turnOffset;

    @Getter
    @Builder.Default
    private PIDSettings driveMotorPIDSettings = new PIDSettings(1, 0, 0);

    @Getter
    @Builder.Default
    // Used to scale the normalized angular error into motor power for the turning motor
    private final double rotationalProportionalGain = 1.6;

    // Drive motor: NEO Vortex on SPARK Flex
    @Getter(lazy = true)
    private final TeamSpark driveMotor = createDriveMotor(driveMotorId, MotorType.kBrushless);

    // Turn motor: NEO on SPARK MAX (unchanged)
    @Getter(lazy = true)
    private final TeamSpark turningMotor = createTurnMotor(turningMotorId, MotorType.kBrushless);

    // Simulation parameters
    @Getter
    @Builder.Default
    private double driveGearRatio = 6.75;

    @Getter
    @Builder.Default
    private double turnGearRatio = 150.0;

    @Getter
    @Builder.Default
    private double driveInertia = 0.025; // kg·m²

    @Getter
    @Builder.Default
    private double turnInertia = 0.004; // kg·m²

    /**
     * Creates a SPARK Flex motor controller instance for drive motors (NEO Vortex).
     */
    private static TeamSpark createDriveMotor(int canId, MotorType type) {
        return (Robot.isReal()) ? new TeamSparkFlexImpl(canId, type) : new TeamSparkFlexSimImpl(canId, type);
    }

    /**
     * Creates a SPARK MAX motor controller instance for turn motors (NEO).
     */
    private static TeamSpark createTurnMotor(int canId, MotorType type) {
        return (Robot.isReal()) ? new TeamSparkMaxImpl(canId, type) : new TeamSparkMaxSimImpl(canId, type);
    }
}
