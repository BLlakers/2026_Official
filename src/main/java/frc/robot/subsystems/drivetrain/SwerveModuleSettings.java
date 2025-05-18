package frc.robot.subsystems.drivetrain;

import frc.robot.support.PIDSettings;
import lombok.Builder;
import lombok.Getter;

@Builder
public class SwerveModuleSettings {

    public static SwerveModuleSettings defaults() {
        return SwerveModuleSettings.builder().build();
    }

    // Name of the SwerveModule
    @Getter
    private String name;

    // CAN ID for the drive motor
    @Getter
    private final int driveMotorChannel;

    // CAN ID for the turning motor
    @Getter
    private final int turningMotorChannel;

    // DIO input for the drive encoder channel B
    @Getter
    private final int turnEncoderPWMChannel;

    // Offset from 0 to 1 for the home position of the encoder
    @Getter
    private final double turnOffset;

    @Getter
    @Builder.Default
    private PIDSettings driveMotorPIDSettings = new PIDSettings(1,0,0);

    @Getter
    @Builder.Default
    // Used to scale the normalized angular error into motor power for the turning motor
    private final double rotationalProportionalGain = 1.6;
}
