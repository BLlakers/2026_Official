package frc.robot.subsystems.drivetrain;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.support.PIDSettings;
import lombok.Builder;
import lombok.Data;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.util.Optional;

@Data
@Builder
public class DrivetrainSettings {

    public static DrivetrainSettings defaults() {
        return DrivetrainSettings.builder().build();
    }

    @Builder.Default
    private double maxSpeed = Units.feetToMeters(12.5); // WP this seemed to work don't know why // 3.68;

    @Builder.Default
    private PIDSettings lateralMovementPIDSettings = new PIDSettings(3, 0, 0);

    @Builder.Default
    private PIDSettings rotationPIDSettings = new PIDSettings(3, 0, 0);

    @Builder.Default
    private SwerveModuleSettings frontLeftSwerveModuleSettings = SwerveModuleSettings.builder()
            .name("Swerve Module/Front Left").driveMotorChannel(Constants.Port.flDriveMtrC)
            .turningMotorChannel(Constants.Port.flSteerMtrC).turnEncoderPWMChannel(Constants.Port.flTurnEncoderDIOC)
            .turnOffset(Constants.RobotVersion2025.flTurnEncoderOffset).build();

    @Builder.Default
    private SwerveModuleSettings frontRightSwerveModuleSettings = SwerveModuleSettings.builder()
            .name("Swerve Module/Front Right").driveMotorChannel(Constants.Port.frDriveMtrC)
            .turningMotorChannel(Constants.Port.frSteerMtrC).turnEncoderPWMChannel(Constants.Port.frTurnEncoderDIOC)
            .turnOffset(Constants.RobotVersion2025.frTurnEncoderOffset).build();

    @Builder.Default
    private SwerveModuleSettings rearLeftSwerveModuleSettings = SwerveModuleSettings.builder()
            .name("Swerve Module/Back Left").driveMotorChannel(Constants.Port.blDriveMtrC)
            .turningMotorChannel(Constants.Port.blSteerMtrC).turnEncoderPWMChannel(Constants.Port.blTurnEncoderDIOC)
            .turnOffset(Constants.RobotVersion2025.blTurnEncoderOffset).build();

    @Builder.Default
    private SwerveModuleSettings rearRightSwerveModuleSettings = SwerveModuleSettings.builder()
            .name("Swerve Module/Back Right").driveMotorChannel(Constants.Port.brDriveMtrC)
            .turningMotorChannel(Constants.Port.brSteerMtrC).turnEncoderPWMChannel(Constants.Port.brTurnEncoderDIOC)
            .turnOffset(Constants.RobotVersion2025.brTurnEncoderOffset).build();

    @Builder.Default
    private double flTurnOffset = Constants.RobotVersion2025.flTurnEncoderOffset;

    @Builder.Default
    private double frTurnOffset = Constants.RobotVersion2025.frTurnEncoderOffset;

    @Builder.Default
    private double blTurnOffset = Constants.RobotVersion2025.blTurnEncoderOffset;

    @Builder.Default
    private double brTurnOffset = Constants.RobotVersion2025.brTurnEncoderOffset;

    @Builder.Default
    private Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    @Builder.Default
    private Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30));

    @Builder.Default
    private SwerveModuleState fullStopAt135Degrees = new SwerveModuleState(0, new Rotation2d(3 * (Math.PI / 4)));

    @Builder.Default
    private SwerveModuleState fullStopAt45Degrees = new SwerveModuleState(0, new Rotation2d((Math.PI / 4)));

    @Builder.Default
    private boolean limelightMegaTag2AlgorithmEnabled = true;

    @Builder.Default
    private int angularVelocityThreshold = 720;

    @Builder.Default
    private double rawFiducialAmbiguityThreshold = 0.7;

    @Builder.Default
    private double rawFiducialDistanceToCameraThreshold = 3.0;

    public double getMaxTurnAngularSpeed() {
        return this.maxSpeed / Constants.Drive.SMBackLeftLocation.getNorm(); // 1/2
    }

    public Optional<RobotConfig> getRobotConfig() {
        try {
            return Optional.of(RobotConfig.fromGUISettings());
        } catch (IOException | ParseException e) {
            // TODO: Correct logging
            System.err.println("Unable to obtain RobotConfig from GUI Settings!");
            return Optional.empty();
        }
    }
}
