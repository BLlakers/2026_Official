package frc.robot.subsystems.drivetrain;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Getter;

@Builder
public class SwerveModuleContext {

    public static SwerveModuleContext defaults() {
        return SwerveModuleContext.builder().build();
    }

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

    @Getter
    private TeamSparkMax driveMotor;

    @Getter
    private TeamSparkMax turningMotor;
}
