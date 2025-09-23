package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkLowLevel;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.support.PIDSettings;
import frc.robot.support.sparkmax.TeamSparkMax;
import frc.robot.support.sparkmax.TeamSparkMaxImpl;
import frc.robot.support.sparkmax.TeamSparkMaxSimImpl;
import lombok.Builder;
import lombok.Data;

import static edu.wpi.first.math.util.Units.feetToMeters;

@Data
@Builder
public class AlgaeMechanismContext {

    /**
     * Instantiates an AlgaeMechanismSettings instance with default values.
     *
     * @return Default AlgaeMechanismSettings
     */
    public static AlgaeMechanismContext defaults() {
        return AlgaeMechanismContext.builder()
                .algaeMotor((Robot.isReal())
                        ? new TeamSparkMaxImpl(Constants.Algae.m_AlgaeMtrC, SparkLowLevel.MotorType.kBrushless)
                        : new TeamSparkMaxSimImpl(Constants.Algae.m_AlgaeMtrC, SparkLowLevel.MotorType.kBrushless))
                .build();
    }

    @Builder.Default
    private Constraints algaeControllerConstraints = new Constraints(feetToMeters(10), feetToMeters(8));

    @Builder.Default
    private PIDSettings algaeControllerPIDSettings = new PIDSettings(.75, 0, 0);

    @Builder.Default
    private PIDSettings algaeMotorPIDSettings = new PIDSettings(1, 0, 0);

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

    @Builder.Default
    private double algaeVelocityConversionFactor = 1;

    // TODO: This channel was never represented within our frc.robot.Constants class. Consider moving the definition
    // of this channel to frc.robot.Constants once initial refactor is complete
    @Builder.Default
    int algaeSensorChannel = 2;

    private TeamSparkMax algaeMotor;

    public double getAlgaePositionConversionFactor() {
        // revolutions -> radians
        return 360 / this.gearRatio;
    }

}
