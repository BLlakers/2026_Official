package frc.robot.subsystems.turrettracker;

/**
 * Turret tracking modes.
 *
 * <ul>
 *   <li>{@link #SHOOTING} — aims at the hub center (default)</li>
 *   <li>{@link #PASSING} — aims at a passing target between the alliance wall
 *       and the hub, offset north or south depending on robot position</li>
 * </ul>
 */
public enum TrackingMode {
    SHOOTING,
    PASSING
}
