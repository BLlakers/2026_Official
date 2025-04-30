package frc.robot.subsystems.drivetrain;

import lombok.Builder;
import lombok.Getter;

@Builder
public class SwerveModuleSettings {

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
}
