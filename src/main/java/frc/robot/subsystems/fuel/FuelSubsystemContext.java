package frc.robot.subsystems.fuel;

import static frc.robot.Constants.FuelConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link FuelSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 */
@Data
@Builder
public class FuelSubsystemContext {

    /**
     * Creates a FuelSubsystemContext with default values
     *
     * @return Default configuration context
     */
    public static FuelSubsystemContext defaults() {
        return FuelSubsystemContext.builder().build();
    }

    /**
     * CAN ID of the feeder roller motor controller
     */
    @Builder.Default
    private final int feederMotorId = FEEDER_MOTOR_ID;

    /**
     * CAN ID of the intake/launcher roller motor controller
     */
    @Builder.Default
    private final int intakeLauncherMotorId = INTAKE_LAUNCHER_MOTOR_ID;

    /**
     * Voltage for feeder roller during intake operation
     */
    @Builder.Default
    private final double intakingFeederVoltage = INTAKING_FEEDER_VOLTAGE;

    /**
     * Voltage for intake/launcher roller during intake operation
     */
    @Builder.Default
    private final double intakingIntakeVoltage = INTAKING_INTAKE_VOLTAGE;

    /**
     * Voltage for feeder roller during launch operation
     */
    @Builder.Default
    private final double launchingFeederVoltage = LAUNCHING_FEEDER_VOLTAGE;

    /**
     * Voltage for launcher roller during launch operation
     */
    @Builder.Default
    private final double launchingLauncherVoltage = LAUNCHING_LAUNCHER_VOLTAGE;

    /**
     * Voltage for feeder roller during spin-up (backs fuel away from launcher)
     */
    @Builder.Default
    private final double spinUpFeederVoltage = SPIN_UP_FEEDER_VOLTAGE;

    /**
     * Current limit for feeder motor in amps
     */
    @Builder.Default
    private final int feederMotorCurrentLimit = FEEDER_MOTOR_CURRENT_LIMIT;

    /**
     * Current limit for launcher motor in amps
     */
    @Builder.Default
    private final int launcherMotorCurrentLimit = LAUNCHER_MOTOR_CURRENT_LIMIT;
}
