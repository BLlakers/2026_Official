package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.support.BuildInfo;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryConfig;
import frc.robot.support.TelemetryLevel;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer = new RobotContainer();
    String codeVersion = BuildInfo.VERSION;
    private PowerDistribution PDH = new PowerDistribution(20, PowerDistribution.ModuleType.kRev);
    public static boolean navxCalibrated = false;
    private boolean isConnected = false;
    private boolean lastBrownedOut = false;

    @Override
    public void close() {
        super.close();
        Telemetry.shutdown();
    }

    @Override
    public void robotInit() {
        Telemetry.initialize(TelemetryConfig.fromDeployDirectory());

        if (m_robotContainer.getLedStrand() != null) {
            m_robotContainer.getLedStrand().changeLed(128, 0, 0);
        }
        try {
            try (UsbCamera cam = CameraServer.startAutomaticCapture()) {
                cam.setResolution(100, 100);
                cam.setFPS(60);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }

        Telemetry.publish("Code Version", codeVersion, TelemetryLevel.MATCH);

        Telemetry.putData(PDH);

        // TODO: Evaluate port forwarding setup
    }

    @Override
    public void robotPeriodic() {
        Telemetry.periodic();

        double batteryVoltage = RobotController.getBatteryVoltage();
        boolean brownedOut = RobotController.isBrownedOut();
        Telemetry.publish("Robot/Battery/VoltageV", batteryVoltage, TelemetryLevel.MATCH);
        Telemetry.publish("Robot/Battery/BrownedOut", brownedOut, TelemetryLevel.MATCH);
        Telemetry.publish("Robot/Battery/VoltageV", batteryVoltage, TelemetryLevel.MATCH);
        Telemetry.publish("Robot/Battery/BrownedOut", brownedOut, TelemetryLevel.MATCH);
        if (brownedOut && !lastBrownedOut) {
            Telemetry.event("Robot/Battery/Brownout", String.format("Voltage=%.2fV", batteryVoltage));
        }
        lastBrownedOut = brownedOut;

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

        // Schedule teleop-init routines (e.g. climb homing) for any enabled subsystems
        m_robotContainer.scheduleTeleopInit();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }
}
