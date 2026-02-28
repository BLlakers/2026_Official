package frc.robot.subsystems.relay;

import static frc.robot.Constants.RelayConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link RelaySubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The relay conveys fuel balls from the intake toward the indexer via a series of six
 * roller bars fitted with soft grabbing pads. All bars are chain-coupled to the innermost bar,
 * which is driven by a single NEO (10:1) on a SparkMax. The mechanism is open-loop only —
 * there is no position sensing.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → rollers convey balls toward the indexer</li>
 *   <li>Negative output → rollers reverse to clear jams</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code relayMotorInverted} — confirm positive output moves balls toward the indexer
 *       during first roller test</li>
 *   <li>{@code relaySpeed} — tune for reliable ball handoff to indexer</li>
 *   <li>{@code relayReverseSpeed} — tune for effective jam clearing</li>
 * </ul>
 */
@Data
@Builder
public class RelaySubsystemContext {

    /**
     * Creates a RelaySubsystemContext with default values from {@link frc.robot.Constants.RelayConstants}.
     *
     * @return Default configuration context
     */
    public static RelaySubsystemContext defaults() {
        return RelaySubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the relay drive motor (NEO on SparkMax). */
    @Builder.Default
    private final int relayMotorId = RELAY_MOTOR_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for the relay motor in amps. */
    @Builder.Default
    private final int relayCurrentLimit = RELAY_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratio
    // -------------------------------------------------------------------------

    /** Gear reduction between NEO shaft and innermost roller bar. 10:1. */
    @Builder.Default
    private final double relayGearRatio = RELAY_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Motor inversion — confirm with build team
    // -------------------------------------------------------------------------

    /**
     * Whether to invert the relay drive motor.
     * Positive output must move balls toward the indexer (upward in the mechanism).
     * <b>TODO: confirm during first roller test.</b>
     */
    @Builder.Default
    private final boolean relayMotorInverted = false;

    // -------------------------------------------------------------------------
    // Speeds [-1.0, 1.0]
    // Convention: positive = convey balls toward indexer
    // -------------------------------------------------------------------------

    /**
     * Speed for conveying balls toward the indexer. Positive.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double relaySpeed = RELAY_SPEED;

    /**
     * Speed for reversing the rollers to clear jams. Negative.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double relayReverseSpeed = RELAY_REVERSE_SPEED;
}
