package frc.robot.subsystems.algae;

import frc.robot.Constants;
import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AlgaeIntakeSettings {

    public static AlgaeIntakeSettings defaults() {
        return AlgaeIntakeSettings.builder().build();
    }

    @Builder.Default
    private final int algaeIntakeMotorChannel = Constants.Algae.intakeMotorChannel;

    @Builder.Default
    private final double advanceIncrement = .75;

    @Builder.Default
    private double reverseIncrement = -.85;
}
