package frc.robot.subsystems.indexer;

import static frc.robot.Constants.IndexerConstants.*;

import lombok.Builder;
import lombok.Data;

/**
 * Configuration context for {@link IndexerSubsystem}.
 * Uses Lombok builder pattern for easy configuration and testing.
 *
 * <h2>Mechanism Overview</h2>
 * <p>The indexer receives fuel balls from the relay and advances them to the shooter via a
 * tower-like chamber. Two grabbing wheels mounted on a shared driveshaft spin in opposite
 * directions to pull the next ball into the void at the center of the tower. Balls queue
 * sequentially and are pushed upward toward the shooter. The driveshaft is driven by a
 * single NEO (1:1) via belt on a SparkMax.
 *
 * <h2>Speed Convention</h2>
 * <ul>
 *   <li>Positive output → wheels pull balls into chamber and advance toward shooter</li>
 *   <li>Negative output → wheels reverse to clear jams</li>
 * </ul>
 *
 * <h2>Build Team TODOs</h2>
 * <ul>
 *   <li>{@code indexerMotorInverted} — confirm positive output pulls balls into the chamber
 *       and advances them toward the shooter during first test</li>
 *   <li>{@code indexerSpeed} — tune for reliable ball advancement without jamming</li>
 *   <li>{@code indexerReverseSpeed} — tune for effective jam clearing</li>
 * </ul>
 */
@Data
@Builder
public class IndexerSubsystemContext {

    /**
     * Creates an IndexerSubsystemContext with default values from
     * {@link frc.robot.Constants.IndexerConstants}.
     *
     * @return Default configuration context
     */
    public static IndexerSubsystemContext defaults() {
        return IndexerSubsystemContext.builder().build();
    }

    // -------------------------------------------------------------------------
    // CAN IDs
    // -------------------------------------------------------------------------

    /** CAN ID of the indexer drive motor (NEO on SparkMax). */
    @Builder.Default
    private final int indexerMotorId = INDEXER_MOTOR_ID;

    // -------------------------------------------------------------------------
    // Current limits
    // -------------------------------------------------------------------------

    /** Smart current limit for the indexer motor in amps. */
    @Builder.Default
    private final int indexerCurrentLimit = INDEXER_CURRENT_LIMIT;

    // -------------------------------------------------------------------------
    // Gear ratio
    // -------------------------------------------------------------------------

    /** Direct drive — NEO shaft to driveshaft. 1:1. */
    @Builder.Default
    private final double indexerGearRatio = INDEXER_GEAR_RATIO;

    // -------------------------------------------------------------------------
    // Motor inversion — confirm with build team
    // -------------------------------------------------------------------------

    /**
     * Whether to invert the indexer drive motor.
     * Positive output must pull balls into the chamber and advance them toward the shooter.
     * The driveshaft geometry makes one wheel spin CW and the other CCW inherently —
     * only the motor's overall direction needs to be set here.
     * <b>TODO: confirm during first indexer test.</b>
     */
    @Builder.Default
    private final boolean indexerMotorInverted = false;

    // -------------------------------------------------------------------------
    // Speeds [-1.0, 1.0]
    // Convention: positive = advance balls toward shooter
    // -------------------------------------------------------------------------

    /**
     * Speed for indexing balls toward the shooter. Positive.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double indexerSpeed = INDEXER_SPEED;

    /**
     * Speed for reversing the indexer to clear jams. Negative.
     * <b>TODO: tune.</b>
     */
    @Builder.Default
    private final double indexerReverseSpeed = INDEXER_REVERSE_SPEED;
}
