package frc.robot.support;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.*;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

/**
 * Enhanced telemetry system for FRC robot data capture and analysis.
 *
 * <p>Features:
 * <ul>
 *   <li>Verbosity levels (NONE, MATCH, LAB, VERBOSE) for controlling capture fidelity</li>
 *   <li>Automatic USB stick detection for portable log storage</li>
 *   <li>Structured type support (Pose2d, ChassisSpeeds, SwerveModuleState[], etc.)</li>
 *   <li>Event markers for significant robot events</li>
 *   <li>Subsystem auto-capture registration</li>
 *   <li>AdvantageScope-compatible output for replay and visualization</li>
 * </ul>
 *
 * <p>Usage:
 * <pre>{@code
 * // In robotInit()
 * Telemetry.initialize(TelemetryConfig.fromDeployDirectory());
 *
 * // In subsystems
 * Telemetry.record("Drivetrain/Pose", pose, TelemetryLevel.MATCH);
 * Telemetry.record("Drivetrain/FL/Current", current, TelemetryLevel.LAB);
 * Telemetry.event("Intake/Acquired", "Coral detected");
 * Telemetry.error("Launcher", "Overcurrent detected", 42.3);
 *
 * // In robotPeriodic()
 * Telemetry.periodic();
 * }</pre>
 *
 * @see TelemetryLevel
 * @see TelemetryConfig
 */
public final class Telemetry {

    // ═══════════════════════════════════════════════════════════════════════════
    // STATE
    // ═══════════════════════════════════════════════════════════════════════════

    private static volatile boolean initialized = false;
    private static volatile TelemetryConfig config;
    private static volatile TelemetryLevel currentLevel = TelemetryLevel.MATCH;

    // Entry registries (thread-safe for potential multi-threaded access)
    private static final Map<String, StringLogEntry> stringLogs = new ConcurrentHashMap<>();
    private static final Map<String, DoubleLogEntry> doubleLogs = new ConcurrentHashMap<>();
    private static final Map<String, FloatLogEntry> floatLogs = new ConcurrentHashMap<>();
    private static final Map<String, IntegerLogEntry> intLogs = new ConcurrentHashMap<>();
    private static final Map<String, BooleanLogEntry> booleanLogs = new ConcurrentHashMap<>();
    private static final Map<String, DoubleArrayLogEntry> doubleArrayLogs = new ConcurrentHashMap<>();
    private static final Map<String, StructLogEntry<?>> structLogs = new ConcurrentHashMap<>();
    private static final Map<String, StructArrayLogEntry<?>> structArrayLogs = new ConcurrentHashMap<>();

    // Subsystem capture registry
    private static final Map<String, Consumer<String>> subsystemCaptures = new ConcurrentHashMap<>();

    // Match state tracking
    private static String lastMatchInfo = "";

    private Telemetry() {
        // Prevent instantiation
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // INITIALIZATION
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Initializes the telemetry system with the given configuration.
     * Should be called once in robotInit().
     *
     * @param telemetryConfig The configuration to use
     */
    public static void initialize(TelemetryConfig telemetryConfig) {
        if (initialized) {
            info("Telemetry already initialized, ignoring duplicate call");
            return;
        }

        config = requireNonNull(telemetryConfig, "config cannot be null");
        currentLevel = config.getLevel();

        // Resolve log path (USB detection happens here)
        String logPath = config.resolveLogPath();

        // Start DataLogManager
        if (logPath != null) {
            DataLogManager.start(logPath);
            info("Telemetry logging to: " + logPath);
        } else {
            DataLogManager.start();
            info("Telemetry logging to default location");
        }

        // Enable NetworkTables logging if configured (enables full replay in AdvantageScope)
        if (config.isNetworkTablesLoggingEnabled()) {
            DataLogManager.logNetworkTables(true);
            info("NetworkTables logging enabled");
        }

        // Publish initial configuration
        publishConfig();

        // Record initialization event
        event("Telemetry/Status", "Initialized at level " + currentLevel.name());

        initialized = true;
    }

    /**
     * Initializes with default configuration (MATCH level, USB enabled).
     */
    public static void initialize() {
        initialize(TelemetryConfig.defaults());
    }

    /**
     * Initializes with a specific level using default configuration otherwise.
     *
     * @param level The telemetry level to use
     */
    public static void initialize(TelemetryLevel level) {
        initialize(new TelemetryConfig.Builder().level(level).build());
    }

    /**
     * Publishes current configuration to NetworkTables for visibility.
     */
    private static void publishConfig() {
        var table = NetworkTableInstance.getDefault().getTable("Telemetry");
        table.getEntry("Level").setString(currentLevel.name());
        table.getEntry("Initialized").setBoolean(true);

        String logPath = config != null ? config.resolveLogPath() : "default";
        table.getEntry("LogPath").setString(logPath != null ? logPath : "default");
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PERIODIC (call from Robot.robotPeriodic)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Periodic update - captures all registered subsystems and checks for level changes.
     * Call this from Robot.robotPeriodic().
     */
    public static void periodic() {
        if (!initialized || currentLevel == TelemetryLevel.NONE) {
            return;
        }

        // Check for runtime level changes via NetworkTables
        if (config != null) {
            TelemetryLevel effective = config.getEffectiveLevel();
            if (effective != currentLevel) {
                TelemetryLevel old = currentLevel;
                currentLevel = effective;
                event("Telemetry/LevelChanged", "Changed from " + old + " to " + currentLevel);
            }
        }

        // Record match state for context
        recordMatchState();

        // Capture all registered subsystems
        for (var entry : subsystemCaptures.entrySet()) {
            try {
                entry.getValue().accept(entry.getKey());
            } catch (Exception e) {
                error(entry.getKey(), "Telemetry capture failed: " + e.getMessage());
            }
        }
    }

    /**
     * Records current match state (mode, time, alliance) for log context.
     */
    private static void recordMatchState() {
        String mode;
        if (DriverStation.isDisabled()) {
            mode = "Disabled";
        } else if (DriverStation.isAutonomous()) {
            mode = "Auto";
        } else if (DriverStation.isTeleop()) {
            mode = "Teleop";
        } else if (DriverStation.isTest()) {
            mode = "Test";
        } else {
            mode = "Unknown";
        }

        String matchInfo = String.format("%s|%.1f|%s",
                mode,
                DriverStation.getMatchTime(),
                DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));

        // Only log when state changes to reduce noise
        if (!matchInfo.equals(lastMatchInfo)) {
            record("Match/Mode", mode, TelemetryLevel.MATCH);
            record("Match/Time", DriverStation.getMatchTime(), TelemetryLevel.MATCH);
            record("Match/Alliance", DriverStation.getAlliance().map(Enum::name).orElse("Unknown"),
                    TelemetryLevel.MATCH);
            lastMatchInfo = matchInfo;
        }
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // SUBSYSTEM REGISTRATION
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Registers a subsystem for automatic telemetry capture during periodic().
     *
     * <p>Example:
     * <pre>{@code
     * // In subsystem constructor
     * Telemetry.registerSubsystem("Drivetrain", this::captureTelemetry);
     *
     * // Capture method
     * private void captureTelemetry(String prefix) {
     *     Telemetry.record(prefix + "/Pose", getPose(), TelemetryLevel.MATCH);
     *     Telemetry.record(prefix + "/Speeds", getSpeeds(), TelemetryLevel.LAB);
     * }
     * }</pre>
     *
     * @param name The subsystem name (used as telemetry key prefix)
     * @param capture Consumer that performs the telemetry capture
     */
    public static void registerSubsystem(String name, Consumer<String> capture) {
        requireNonNull(name, "name cannot be null");
        requireNonNull(capture, "capture cannot be null");
        subsystemCaptures.put(name, capture);
        event("Telemetry/Registered", name);
    }

    /**
     * Unregisters a subsystem from automatic capture.
     *
     * @param name The subsystem name to unregister
     */
    public static void unregisterSubsystem(String name) {
        subsystemCaptures.remove(name);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // PRIMITIVE RECORDING (Level-aware)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Records a double value if the specified level permits.
     *
     * @param key The telemetry key (e.g., "Drivetrain/Speed")
     * @param value The value to record
     * @param level The minimum level required to record this data
     */
    public static void record(String key, double value, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        DoubleLogEntry entry = doubleLogs.computeIfAbsent(key,
                k -> new DoubleLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    /**
     * Records an int value if the specified level permits.
     */
    public static void record(String key, int value, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        IntegerLogEntry entry = intLogs.computeIfAbsent(key,
                k -> new IntegerLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    /**
     * Records a float value if the specified level permits.
     */
    public static void record(String key, float value, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        FloatLogEntry entry = floatLogs.computeIfAbsent(key,
                k -> new FloatLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    /**
     * Records a boolean value if the specified level permits.
     */
    public static void record(String key, boolean value, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        BooleanLogEntry entry = booleanLogs.computeIfAbsent(key,
                k -> new BooleanLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    /**
     * Records a String value if the specified level permits.
     */
    public static void record(String key, String value, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        requireNonNull(value, "value cannot be null");
        StringLogEntry entry = stringLogs.computeIfAbsent(key,
                k -> new StringLogEntry(DataLogManager.getLog(), k));
        entry.append(value);
    }

    /**
     * Records a double array if the specified level permits.
     */
    public static void record(String key, double[] values, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        requireNonNull(values, "values cannot be null");
        DoubleArrayLogEntry entry = doubleArrayLogs.computeIfAbsent(key,
                k -> new DoubleArrayLogEntry(DataLogManager.getLog(), k));
        entry.append(values);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // STRUCTURED TYPE RECORDING (WPILib structs for AdvantageScope)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Records a Pose2d if the specified level permits.
     * Recorded as a WPILib struct for native AdvantageScope visualization.
     */
    public static void record(String key, Pose2d pose, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (pose == null) return;
        recordStruct(key, Pose2d.struct, pose);
    }

    /**
     * Records a Pose3d if the specified level permits.
     */
    public static void record(String key, Pose3d pose, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (pose == null) return;
        recordStruct(key, Pose3d.struct, pose);
    }

    /**
     * Records a Rotation2d if the specified level permits.
     */
    public static void record(String key, Rotation2d rotation, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (rotation == null) return;
        recordStruct(key, Rotation2d.struct, rotation);
    }

    /**
     * Records ChassisSpeeds if the specified level permits.
     */
    public static void record(String key, ChassisSpeeds speeds, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (speeds == null) return;
        recordStruct(key, ChassisSpeeds.struct, speeds);
    }

    /**
     * Records a SwerveModuleState array if the specified level permits.
     * Enables swerve visualization in AdvantageScope.
     */
    public static void record(String key, SwerveModuleState[] states, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (states == null) return;
        recordStructArray(key, SwerveModuleState.struct, states);
    }

    /**
     * Records a Pose2d array if the specified level permits.
     * Useful for trajectory visualization.
     */
    public static void recordPoses(String key, Pose2d[] poses, TelemetryLevel level) {
        if (!shouldLog(level)) return;
        requireNonNull(key, "key cannot be null");
        if (poses == null) return;
        recordStructArray(key, Pose2d.struct, poses);
    }

    @SuppressWarnings("unchecked")
    private static <T> void recordStruct(String key, Struct<T> struct, T value) {
        StructLogEntry<T> entry = (StructLogEntry<T>) structLogs.computeIfAbsent(key,
                k -> new StructLogEntry<>(DataLogManager.getLog(), k, struct));
        entry.append(value);
    }

    @SuppressWarnings("unchecked")
    private static <T> void recordStructArray(String key, Struct<T> struct, T[] values) {
        StructArrayLogEntry<T> entry = (StructArrayLogEntry<T>) structArrayLogs.computeIfAbsent(key,
                k -> new StructArrayLogEntry<>(DataLogManager.getLog(), k, struct));
        entry.append(values);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // EVENT AND ERROR LOGGING (Always captured regardless of level)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Logs a significant event. Events are ALWAYS captured regardless of level.
     * Use for: command starts/ends, state transitions, game piece acquisitions.
     *
     * @param key The event category (e.g., "Intake/Acquired", "Auto/PathStarted")
     * @param description Human-readable description of the event
     */
    public static void event(String key, String description) {
        if (currentLevel == TelemetryLevel.NONE) return;
        requireNonNull(key, "key cannot be null");
        String msg = description != null ? description : "";

        StringLogEntry entry = stringLogs.computeIfAbsent(key,
                k -> new StringLogEntry(DataLogManager.getLog(), k));
        entry.append(msg);

        // Also log to console for visibility during testing
        DataLogManager.log("[EVENT] " + key + ": " + msg);
    }

    /**
     * Logs an event with timestamp.
     */
    public static void event(String key) {
        event(key, String.format("@%.3f", Timer.getFPGATimestamp()));
    }

    /**
     * Logs an error event. Errors are ALWAYS captured and include additional context.
     *
     * @param subsystem The subsystem where the error occurred
     * @param message Error description
     */
    public static void error(String subsystem, String message) {
        if (currentLevel == TelemetryLevel.NONE) return;
        requireNonNull(subsystem, "subsystem cannot be null");
        String key = subsystem + "/Error";
        String fullMessage = String.format("[%.3f] %s", Timer.getFPGATimestamp(), message);

        StringLogEntry entry = stringLogs.computeIfAbsent(key,
                k -> new StringLogEntry(DataLogManager.getLog(), k));
        entry.append(fullMessage);

        // Log to console and DriverStation
        DataLogManager.log("[ERROR] " + subsystem + ": " + message);
        DriverStation.reportError("[" + subsystem + "] " + message, false);
    }

    /**
     * Logs an error with an associated numeric value (e.g., overcurrent value).
     */
    public static void error(String subsystem, String message, double value) {
        error(subsystem, String.format("%s (value=%.3f)", message, value));
    }

    /**
     * Logs an error with an exception.
     */
    public static void error(String subsystem, String message, Throwable cause) {
        String full = message;
        if (cause != null) {
            full += " | " + cause.getClass().getSimpleName() + ": " + cause.getMessage();
        }
        error(subsystem, full);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // LEGACY API (backward compatible with existing code)
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Records a string value at MATCH level (legacy API).
     * @deprecated Use {@link #record(String, String, TelemetryLevel)} instead
     */
    public static void record(String key, String value) {
        record(key, value, TelemetryLevel.MATCH);
    }

    /**
     * Records a float value at MATCH level (legacy API).
     * @deprecated Use {@link #record(String, float, TelemetryLevel)} instead
     */
    public static void record(String key, float value) {
        record(key, value, TelemetryLevel.MATCH);
    }

    /**
     * Records a double value at MATCH level (legacy API).
     * @deprecated Use {@link #record(String, double, TelemetryLevel)} instead
     */
    public static void record(String key, double value) {
        record(key, value, TelemetryLevel.MATCH);
    }

    /**
     * Records an int value at MATCH level (legacy API).
     * @deprecated Use {@link #record(String, int, TelemetryLevel)} instead
     */
    public static void record(String key, int value) {
        record(key, value, TelemetryLevel.MATCH);
    }

    /**
     * Logs an info message to the data log (legacy API).
     */
    public static void info(String value) {
        requireNonNull(value, "value cannot be null");
        DataLogManager.log(value);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // UTILITY METHODS
    // ═══════════════════════════════════════════════════════════════════════════

    /**
     * Returns the current telemetry level.
     */
    public static TelemetryLevel getCurrentLevel() {
        return currentLevel;
    }

    /**
     * Sets the telemetry level at runtime.
     */
    public static void setLevel(TelemetryLevel level) {
        requireNonNull(level, "level cannot be null");
        if (level != currentLevel) {
            TelemetryLevel old = currentLevel;
            currentLevel = level;
            event("Telemetry/LevelChanged", "Changed from " + old + " to " + level);

            // Update NetworkTables
            NetworkTableInstance.getDefault()
                    .getTable("Telemetry")
                    .getEntry("Level")
                    .setString(level.name());
        }
    }

    /**
     * Returns whether the system has been initialized.
     */
    public static boolean isInitialized() {
        return initialized;
    }

    /**
     * Checks if a record at the given level should be logged.
     */
    private static boolean shouldLog(TelemetryLevel level) {
        return level.shouldLog(currentLevel);
    }

    /**
     * Shuts down the telemetry system, flushing all pending data.
     * Call from Robot.close().
     */
    public static void shutdown() {
        if (initialized) {
            event("Telemetry/Status", "Shutting down");
            DataLogManager.stop();
            initialized = false;
        }
    }

    /**
     * Clears all entry registries. Primarily for testing.
     */
    static void reset() {
        stringLogs.clear();
        doubleLogs.clear();
        floatLogs.clear();
        intLogs.clear();
        booleanLogs.clear();
        doubleArrayLogs.clear();
        structLogs.clear();
        structArrayLogs.clear();
        subsystemCaptures.clear();
        initialized = false;
        config = null;
        currentLevel = TelemetryLevel.MATCH;
        lastMatchInfo = "";
    }
}
