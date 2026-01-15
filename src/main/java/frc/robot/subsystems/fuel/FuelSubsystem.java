package frc.robot.subsystems.fuel;

import static java.util.Objects.requireNonNull;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Fuel subsystem for intake and launching game pieces.
 * Adapted from KitBot CANFuelSubsystem reference implementation with Context pattern.
 *
 * <p>Features:
 * <ul>
 *   <li>Two brushed motor rollers (feeder and intake/launcher)</li>
 *   <li>Voltage-based control for simplicity</li>
 *   <li>SmartDashboard integration for runtime tuning</li>
 *   <li>Command factory methods following WPILib best practices</li>
 *   <li>Context-based configuration for testability</li>
 * </ul>
 */
public class FuelSubsystem extends SubsystemBase {

    private final FuelSubsystemContext context;
    private final SparkMax feederRoller;
    private final SparkMax intakeLauncherRoller;

    /**
     * Instantiates a new FuelSubsystem with default {@link FuelSubsystemContext}
     */
    public FuelSubsystem() {
        this(FuelSubsystemContext.defaults());
    }

    /**
     * Instantiates a new FuelSubsystem with the specified context
     *
     * @param context The FuelSubsystemContext to apply to this instance
     */
    public FuelSubsystem(final FuelSubsystemContext context) {
        requireNonNull(context, "FuelSubsystemContext cannot be null");
        this.context = context;

        // Create brushed motors for each of the motors on the fuel mechanism
        this.feederRoller = new SparkMax(this.context.getFeederMotorId(), MotorType.kBrushed);
        this.intakeLauncherRoller = new SparkMax(this.context.getIntakeLauncherMotorId(), MotorType.kBrushed);

        // Put default values for various fuel operations onto the dashboard
        // All methods in this subsystem pull their values from the dashboard to allow
        // you to tune the values easily, and then replace the values in Constants.java
        // with your new values after testing
        SmartDashboard.putNumber("Fuel/Intaking feeder voltage", this.context.getIntakingFeederVoltage());
        SmartDashboard.putNumber("Fuel/Intaking intake voltage", this.context.getIntakingIntakeVoltage());
        SmartDashboard.putNumber("Fuel/Launching feeder voltage", this.context.getLaunchingFeederVoltage());
        SmartDashboard.putNumber("Fuel/Launching launcher voltage", this.context.getLaunchingLauncherVoltage());
        SmartDashboard.putNumber("Fuel/Spin-up feeder voltage", this.context.getSpinUpFeederVoltage());

        configureMotors();
    }

    /**
     * Configures both motor controllers with current limits and safe parameters
     */
    private void configureMotors() {
        // Configure feeder roller
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(this.context.getFeederMotorCurrentLimit());
        this.feederRoller.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Configure launcher roller (inverted so positive values work for both intake and launch)
        SparkMaxConfig launcherConfig = new SparkMaxConfig();
        launcherConfig.inverted(true);
        launcherConfig.smartCurrentLimit(this.context.getLauncherMotorCurrentLimit());
        this.intakeLauncherRoller.configure(
                launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Sets the rollers to values for intaking fuel.
     * Values are pulled from SmartDashboard for runtime tuning.
     */
    private void intake() {
        feederRoller.setVoltage(
                SmartDashboard.getNumber("Fuel/Intaking feeder voltage", this.context.getIntakingFeederVoltage()));
        intakeLauncherRoller.setVoltage(
                SmartDashboard.getNumber("Fuel/Intaking intake voltage", this.context.getIntakingIntakeVoltage()));
    }

    /**
     * Sets the rollers to values for ejecting fuel out the intake.
     * Uses the same values as intaking, but in the opposite direction.
     */
    private void eject() {
        feederRoller.setVoltage(
                -1 * SmartDashboard.getNumber("Fuel/Intaking feeder voltage", this.context.getIntakingFeederVoltage()));
        intakeLauncherRoller.setVoltage(
                -1 * SmartDashboard.getNumber("Fuel/Intaking intake voltage", this.context.getIntakingIntakeVoltage()));
    }

    /**
     * Sets the rollers to values for launching fuel.
     * Values are pulled from SmartDashboard for runtime tuning.
     */
    private void launch() {
        feederRoller.setVoltage(
                SmartDashboard.getNumber("Fuel/Launching feeder voltage", this.context.getLaunchingFeederVoltage()));
        intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
                "Fuel/Launching launcher voltage", this.context.getLaunchingLauncherVoltage()));
    }

    /**
     * Stops all rollers
     */
    private void stop() {
        feederRoller.set(0);
        intakeLauncherRoller.set(0);
    }

    /**
     * Spins up the launcher roller while spinning the feeder roller backwards
     * to push fuel away from the launcher (prevents premature feeding)
     */
    private void spinUp() {
        feederRoller.setVoltage(
                SmartDashboard.getNumber("Fuel/Spin-up feeder voltage", this.context.getSpinUpFeederVoltage()));
        intakeLauncherRoller.setVoltage(SmartDashboard.getNumber(
                "Fuel/Launching launcher voltage", this.context.getLaunchingLauncherVoltage()));
    }

    /**
     * Command factory to create an intake command.
     * Runs intake() continuously until interrupted, then stops.
     *
     * @return Command that intakes fuel
     */
    public Command getIntakeCommand() {
        return this.runEnd(this::intake, this::stop);
    }

    /**
     * Command factory to create an eject command.
     * Runs eject() continuously until interrupted, then stops.
     *
     * @return Command that ejects fuel
     */
    public Command getEjectCommand() {
        return this.runEnd(this::eject, this::stop);
    }

    /**
     * Command factory to create a launch command.
     * Runs launch() continuously until interrupted, then stops.
     *
     * @return Command that launches fuel
     */
    public Command getLaunchCommand() {
        return this.run(this::launch);
    }

    /**
     * Command factory to create a spin-up command.
     * Spins up the launcher while backing fuel away.
     *
     * @return Command that spins up the launcher
     */
    public Command getSpinUpCommand() {
        return this.run(this::spinUp);
    }

    /**
     * Command factory to create a stop command.
     *
     * @return Command that stops all rollers
     */
    public Command getStopCommand() {
        return this.runOnce(this::stop);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // Add any periodic status updates or telemetry here if needed
    }
}
