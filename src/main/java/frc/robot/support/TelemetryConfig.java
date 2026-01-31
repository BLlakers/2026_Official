package frc.robot.support;

import static java.util.Objects.requireNonNull;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;

/**
 * Configuration for the telemetry system. Supports multiple configuration sources
 * with the following priority (highest to lowest):
 * <ol>
 *   <li>NetworkTables entry "Telemetry/Level" (allows runtime changes)</li>
 *   <li>Properties file at deploy directory (telemetry.properties)</li>
 *   <li>Builder defaults</li>
 * </ol>
 *
 * <p>Example telemetry.properties file:
 * <pre>
 * telemetry.level=LAB
 * telemetry.usb.enabled=true
 * telemetry.networktables.enabled=true
 * </pre>
 */
public final class TelemetryConfig {

    /** Default USB paths to check on roboRIO (in order of preference) */
    private static final String[] USB_PATHS = {
        "/media/sda1", // Primary USB mount on roboRIO 2
        "/U", // Alternate USB mount point
        "/media/sdb1" // Secondary USB if multiple connected
    };

    /** Subdirectory name for FRC logs on USB */
    private static final String LOG_SUBDIRECTORY = "FRC_LOGS";

    private final TelemetryLevel level;
    private final boolean usbEnabled;
    private final boolean networkTablesLoggingEnabled;
    private final String customLogPath;

    private TelemetryConfig(Builder builder) {
        this.level = builder.level;
        this.usbEnabled = builder.usbEnabled;
        this.networkTablesLoggingEnabled = builder.networkTablesLoggingEnabled;
        this.customLogPath = builder.customLogPath;
    }

    /**
     * Creates a default configuration suitable for competition matches.
     *
     * @return A TelemetryConfig with MATCH level and USB enabled
     */
    public static TelemetryConfig defaults() {
        return new Builder().build();
    }

    /**
     * Loads configuration from the deploy directory's telemetry.properties file,
     * falling back to defaults if the file doesn't exist.
     *
     * @return A TelemetryConfig loaded from file or defaults
     */
    public static TelemetryConfig fromDeployDirectory() {
        File propsFile = new File(Filesystem.getDeployDirectory(), "telemetry.properties");
        if (propsFile.exists()) {
            try {
                return fromPropertiesFile(propsFile);
            } catch (IOException e) {
                System.err.println("[Telemetry] Failed to load telemetry.properties: " + e.getMessage());
            }
        }
        return defaults();
    }

    /**
     * Loads configuration from a properties file.
     *
     * @param file The properties file to load
     * @return A TelemetryConfig from the file
     * @throws IOException If the file cannot be read
     */
    public static TelemetryConfig fromPropertiesFile(File file) throws IOException {
        Properties props = new Properties();
        try (FileInputStream fis = new FileInputStream(file)) {
            props.load(fis);
        }

        Builder builder = new Builder();

        String levelStr = props.getProperty("telemetry.level");
        if (levelStr != null) {
            builder.level(TelemetryLevel.fromString(levelStr));
        }

        String usbStr = props.getProperty("telemetry.usb.enabled");
        if (usbStr != null) {
            builder.usbEnabled(Boolean.parseBoolean(usbStr));
        }

        String ntStr = props.getProperty("telemetry.networktables.enabled");
        if (ntStr != null) {
            builder.networkTablesLoggingEnabled(Boolean.parseBoolean(ntStr));
        }

        String customPath = props.getProperty("telemetry.path");
        if (customPath != null && !customPath.isBlank()) {
            builder.customLogPath(customPath);
        }

        return builder.build();
    }

    /**
     * Gets the effective telemetry level, checking NetworkTables for runtime override.
     *
     * @return The telemetry level to use
     */
    public TelemetryLevel getEffectiveLevel() {
        // Check for runtime override via NetworkTables
        String ntLevel = NetworkTableInstance.getDefault()
                .getTable("Telemetry")
                .getEntry("Level")
                .getString(null);

        if (ntLevel != null) {
            TelemetryLevel override = TelemetryLevel.fromString(ntLevel);
            if (override != level) {
                return override;
            }
        }
        return level;
    }

    /**
     * Gets the configured telemetry level (without checking runtime overrides).
     *
     * @return The configured level
     */
    public TelemetryLevel getLevel() {
        return level;
    }

    /**
     * Returns whether USB logging is enabled.
     *
     * @return true if USB logging should be attempted
     */
    public boolean isUsbEnabled() {
        return usbEnabled;
    }

    /**
     * Returns whether NetworkTables data should also be logged to the data log.
     *
     * @return true if NT logging is enabled
     */
    public boolean isNetworkTablesLoggingEnabled() {
        return networkTablesLoggingEnabled;
    }

    /**
     * Determines the best log path based on configuration and available storage.
     *
     * <p>Priority:
     * <ol>
     *   <li>Custom path if specified and writable</li>
     *   <li>USB stick if enabled and detected</li>
     *   <li>null (use DataLogManager default: /home/lvuser/logs on roboRIO)</li>
     * </ol>
     *
     * @return The log directory path, or null to use default
     */
    public String resolveLogPath() {
        // Custom path takes precedence
        if (customLogPath != null && !customLogPath.isBlank()) {
            File customDir = new File(customLogPath);
            if (ensureDirectoryExists(customDir)) {
                return customLogPath;
            }
            System.err.println("[Telemetry] Custom path not writable: " + customLogPath);
        }

        // Try USB if enabled
        if (usbEnabled) {
            String usbPath = detectUsbPath();
            if (usbPath != null) {
                return usbPath;
            }
        }

        // Fall back to default (null signals DataLogManager to use its default)
        return null;
    }

    /**
     * Attempts to detect a connected USB stick and returns the log directory path.
     *
     * @return Path to USB log directory, or null if no USB detected
     */
    private String detectUsbPath() {
        // In simulation, don't try USB paths
        if (RobotBase.isSimulation()) {
            return null;
        }

        for (String basePath : USB_PATHS) {
            File baseDir = new File(basePath);
            if (baseDir.exists() && baseDir.isDirectory()) {
                File logDir = new File(baseDir, LOG_SUBDIRECTORY);
                if (ensureDirectoryExists(logDir)) {
                    System.out.println("[Telemetry] USB detected at: " + logDir.getAbsolutePath());
                    return logDir.getAbsolutePath();
                }
            }
        }

        System.out.println("[Telemetry] No USB storage detected, using default log location");
        return null;
    }

    /**
     * Ensures a directory exists, creating it if necessary.
     *
     * @param dir The directory to check/create
     * @return true if directory exists and is writable
     */
    private boolean ensureDirectoryExists(File dir) {
        if (dir.exists()) {
            return dir.isDirectory() && dir.canWrite();
        }
        try {
            return dir.mkdirs();
        } catch (SecurityException e) {
            return false;
        }
    }

    @Override
    public String toString() {
        return String.format(
                "TelemetryConfig[level=%s, usb=%s, ntLogging=%s, customPath=%s]",
                level, usbEnabled, networkTablesLoggingEnabled, customLogPath);
    }

    /**
     * Builder for TelemetryConfig.
     */
    public static final class Builder {
        private TelemetryLevel level = TelemetryLevel.MATCH;
        private boolean usbEnabled = true;
        private boolean networkTablesLoggingEnabled = true;
        private String customLogPath = null;

        public Builder() {}

        /**
         * Sets the telemetry verbosity level.
         *
         * @param level The level to use
         * @return This builder
         */
        public Builder level(TelemetryLevel level) {
            this.level = requireNonNull(level, "level cannot be null");
            return this;
        }

        /**
         * Sets whether to attempt USB logging.
         *
         * @param enabled true to enable USB detection
         * @return This builder
         */
        public Builder usbEnabled(boolean enabled) {
            this.usbEnabled = enabled;
            return this;
        }

        /**
         * Sets whether to log NetworkTables data to the data log.
         * This enables replay of SmartDashboard values in AdvantageScope.
         *
         * @param enabled true to enable NT logging
         * @return This builder
         */
        public Builder networkTablesLoggingEnabled(boolean enabled) {
            this.networkTablesLoggingEnabled = enabled;
            return this;
        }

        /**
         * Sets a custom log path (overrides USB detection).
         *
         * @param path The custom directory path
         * @return This builder
         */
        public Builder customLogPath(String path) {
            this.customLogPath = path;
            return this;
        }

        /**
         * Builds the TelemetryConfig.
         *
         * @return A new TelemetryConfig instance
         */
        public TelemetryConfig build() {
            return new TelemetryConfig(this);
        }
    }
}
