package frc.robot.subsystems.template;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link TemplateMechanism}.
 * Uses Lombok builder pattern for easy configuration and testing.
 */
@Data
@Builder
public class TemplateMechanismContext {

    /**
     * Creates a TemplateMechanismContext with default values
     *
     * @return Default configuration context
     */
    public static TemplateMechanismContext defaults() {
        return TemplateMechanismContext.builder().build();
    }

    /**
     * CAN ID of the TalonFX motor controller
     */
    @Builder.Default
    private final int motorId = 0;  // TODO: Set actual CAN ID in Constants

    /**
     * CAN bus name (use "rio" for roboRIO CAN bus, or specific CANivore name)
     */
    @Builder.Default
    private final String canBus = "rio";

    /**
     * Speed for advancing the mechanism (0.0 to 1.0)
     */
    @Builder.Default
    private final double advanceSpeed = 0.5;

    /**
     * Speed for reversing the mechanism (-1.0 to 0.0)
     */
    @Builder.Default
    private final double reverseSpeed = -0.5;

    /**
     * Supply current limit in amps
     */
    @Builder.Default
    private final double currentLimit = 40.0;
}
