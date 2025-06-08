package frc.robot.subsystems.algae;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.support.PIDSettings;
import lombok.Builder;
import lombok.Data;

import static edu.wpi.first.math.util.Units.feetToMeters;

@Data
@Builder
public class AlgaeMechanismSettings {

    /**
     * Instantiates an AlgaeMechanismSettings instance with default values.
     *
     * @return Default AlgaeMechanismSettings
     */
    public static AlgaeMechanismSettings defaults() {
        return AlgaeMechanismSettings.builder().build();
    }

    @Builder.Default
    private Constraints algaeControllerConstraints = new Constraints(feetToMeters(10), feetToMeters(8));

    @Builder.Default
    private PIDSettings algeaControllerPIDSettings = new PIDSettings(.75, 0, 0);

    @Builder.Default
    private PIDSettings algeaMotorPIDSettings = new PIDSettings(1, 0, 0);

    @Builder.Default
    private double algaeUpPosition = -1.03;

    @Builder.Default
    private double algaeDownPosition = -0.14;

    @Builder.Default
    private double algaeMiddlePosition = -0.75;

    @Builder.Default
    private double algaeGroundPosition = -1.165;

    @Builder.Default
    private double algaeControllerTolerance = 0.035;

    @Builder.Default
    private final double advanceIncrement = .1;

    @Builder.Default
    private double reverseIncrement = -.1;

    @Builder.Default
    private final double gearRatio = 75;

    @Builder.Default
    private int countsPerRevolution = 8192;

    @Builder.Default
    private int algaeSensorForwardIRValue = 2400;

    // TODO: This channel was never represented within our frc.robot.Constants class. Consider moving the definition
    // of this channel to frc.robot.Constants once initial refactor is complete
    @Builder.Default
    private int algaeSensorChannel = 2;

    @Builder.Default
    private double algaeVelocityConversionFactor = 1;

    public double getAlgaePositionConversionFactor() {
        // revolutions -> radians
        return 360 / this.gearRatio;
    }

}
