package frc.robot.support;

/**
 * Defines telemetry verbosity levels for controlling data capture fidelity.
 *
 * <p>Use different levels based on the context:
 * <ul>
 *   <li>{@link #NONE} - No telemetry capture (silent mode for critical situations)</li>
 *   <li>{@link #MATCH} - Essential data for competition matches (~20 Hz effective rate)</li>
 *   <li>{@link #LAB} - Detailed data for practice and lab sessions (~50 Hz)</li>
 *   <li>{@link #VERBOSE} - Maximum detail for deep debugging (~100+ Hz)</li>
 * </ul>
 *
 * <p>Example usage:
 * <pre>{@code
 * // Only log if current level permits LAB-level data
 * Telemetry.record("Drivetrain/FL/Current", motorCurrent, TelemetryLevel.LAB);
 *
 * // Essential data always logged at MATCH level or above
 * Telemetry.record("Drivetrain/Pose", pose, TelemetryLevel.MATCH);
 * }</pre>
 */
public enum TelemetryLevel {

    /**
     * No telemetry capture. Use when telemetry overhead must be eliminated entirely.
     */
    NONE(0),

    /**
     * Competition match level. Captures essential data needed for post-match analysis:
     * robot pose, command states, errors, and key mechanism states.
     * Recommended for qualification and elimination matches.
     */
    MATCH(1),

    /**
     * Lab/practice level. Captures MATCH data plus motor currents, raw sensor values,
     * and detailed subsystem states. Recommended for practice sessions and tuning.
     */
    LAB(2),

    /**
     * Maximum verbosity. Captures LAB data plus PID internals, all sensor signals,
     * and high-frequency data points. Use for deep debugging specific issues.
     * Warning: May generate large log files.
     */
    VERBOSE(3);

    private final int priority;

    TelemetryLevel(int priority) {
        this.priority = priority;
    }

    /**
     * Returns the numeric priority of this level. Higher values = more verbose.
     *
     * @return The priority value (0-3)
     */
    public int getPriority() {
        return priority;
    }

    /**
     * Determines if a record at this level should be logged given the configured level.
     *
     * <p>A record is logged if its level priority is less than or equal to the configured level.
     * For example, a MATCH-level record (priority 1) will be logged when configured for LAB (priority 2).
     *
     * @param configuredLevel The currently configured telemetry level
     * @return true if this level should be logged, false otherwise
     */
    public boolean shouldLog(TelemetryLevel configuredLevel) {
        if (configuredLevel == NONE) {
            return false;
        }
        return this.priority <= configuredLevel.priority;
    }

    /**
     * Parses a string to a TelemetryLevel, defaulting to MATCH if invalid.
     *
     * @param value The string value to parse (case-insensitive)
     * @return The corresponding TelemetryLevel, or MATCH if not recognized
     */
    public static TelemetryLevel fromString(String value) {
        if (value == null || value.isBlank()) {
            return MATCH;
        }
        try {
            return valueOf(value.toUpperCase().trim());
        } catch (IllegalArgumentException e) {
            return MATCH;
        }
    }
}
