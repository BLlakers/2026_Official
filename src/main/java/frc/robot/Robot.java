package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryConfig;
import frc.robot.support.TelemetryLevel;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = new RobotContainer();
    String codeVersion = "0.0";
    private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);
    public static boolean navxCalibrated = false;
    private boolean isConnected = false;

    @Override
    public void close() {
        super.close();
        Telemetry.shutdown();
    }

    // commit
    @Override
    public void robotInit() {
        // Initialize telemetry first - attempts USB detection, falls back to default location
        // Configuration can be overridden via telemetry.properties in deploy directory
        Telemetry.initialize(TelemetryConfig.fromDeployDirectory());

        m_robotContainer.getLedStrand().changeLed(128, 0, 0);
        try {
            try (UsbCamera cam = CameraServer.startAutomaticCapture()) {
                cam.setResolution(100, 100);
                cam.setFPS(60);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        Telemetry.publish("Code Version", codeVersion, TelemetryLevel.MATCH);

        // Register PDH as a Sendable (only needs to be done once)
        Telemetry.putData(PDH);

        // TODO: Evaluate port forwarding setup
    }

    @Override
    public void robotPeriodic() {
        // Capture telemetry from all registered subsystems
        Telemetry.periodic();

        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        m_robotContainer.getDriveTrain().setFieldRelativeEnable(false);
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void teleopInit() {
        m_robotContainer.getDriveTrain().setFieldRelativeEnable(true);
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
