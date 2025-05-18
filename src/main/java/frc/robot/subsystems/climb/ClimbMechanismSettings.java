package frc.robot.subsystems.climb;

import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class ClimbMechanismSettings {

    /**
     * Instantiates an ClimbMechanismSettings instance with default values.
     *
     * @return Default ClimbMechanismSettings
     */
    public static ClimbMechanismSettings defaults() {
        return ClimbMechanismSettings.builder().build();
    }

    @Builder.Default
    private int climbMotorChannel = Constants.Port.climbMotoChannel;

    @Builder.Default
    private int climbMagSwitchChannel = Constants.Port.climbMagSwitchDIOC;

    @Builder.Default
    private PIDSettings climbControllerPIDSettings = new PIDSettings(1, 0, 0);

    @Builder.Default
    private double climbPositionConversionFactor = 1;

    @Builder.Default
    private double climbVelocityConversionFactor = 1;

    @Builder.Default
    private final double advanceIncrement = .75;

    @Builder.Default
    private double reverseIncrement = -.75;
}
