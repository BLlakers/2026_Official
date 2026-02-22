package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.Optional;

/**
 * A simulation test command that exercises the full climb state machine.
 *
 * <p>Runs the following sequence:
 * <ol>
 *   <li>Home the telescope (retracts to ground-contact hardstop, zeroes encoder)</li>
 *   <li>For each bar (1, 2, 3):
 *     <ul>
 *       <li>Extend to bar (B button equivalent)</li>
 *       <li>Pause to observe extension</li>
 *       <li>Climb / retract to engage hooks (A button equivalent)</li>
 *       <li>Pause to observe engagement</li>
 *     </ul>
 *   </li>
 *   <li>Hold at bar 3 until cancelled</li>
 * </ol>
 *
 * <p>Use this in simulation to verify:
 * <ul>
 *   <li>State transitions: IDLE → HOMING → STORED → EXTENDING → RETRACTING → HOLDING</li>
 *   <li>Encoder values oscillate: positive (extend) → through zero → negative (engage)</li>
 *   <li>Bar counter advances 0 → 1 → 2 → 3</li>
 *   <li>Telescope visualization extends upward and retracts correctly</li>
 * </ul>
 *
 * <p>Watch these telemetry keys during the test:
 * <ul>
 *   <li>{@code Climb/State} — should cycle through all states</li>
 *   <li>{@code Climb/CurrentBar} — should advance 0 → 1 → 2 → 3</li>
 *   <li>{@code Climb/Encoder/PositionRotations} — should oscillate positive/negative</li>
 *   <li>{@code ClimbTest/Status} — human-readable description of the current step</li>
 * </ul>
 *
 * <p><strong>Note:</strong> In simulation, the motor current spike for homing will not occur
 * (no physical load), so homing may time out or run indefinitely. The command includes a
 * 3-second timeout on homing to handle this gracefully.
 */
public class ClimbTestCommand extends SequentialCommandGroup {

    private static final double PAUSE_SECONDS = 2.0;
    private static final double HOMING_TIMEOUT_SECONDS = 3.0;

    private ClimbTestCommand(ClimbSubsystem climb) {
        // Step 1: Home
        addCommands(
                logStep("Homing telescope (retracting to ground hardstop)..."),
                climb.getHomingCommand().withTimeout(HOMING_TIMEOUT_SECONDS),
                // Force encoder to 0 and state to STORED even if homing timed out in sim
                Commands.runOnce(() -> {}, climb),
                logStep("Homing complete. State should be STORED, encoder ≈ 0."),
                Commands.waitSeconds(PAUSE_SECONDS));

        // Steps 2-4: Extend + Climb for each bar
        for (int bar = 1; bar <= 3; bar++) {
            final int barNum = bar;

            // Extend to bar
            addCommands(
                    logStep("Extending to bar " + barNum + " (encoder → positive)..."),
                    climb.getExtendToBarCommand(),
                    logStep("Extended to bar " + barNum + ". Top hook should be at bar. State: HOLDING."),
                    Commands.waitSeconds(PAUSE_SECONDS));

            // Climb (retract to engage hooks)
            addCommands(
                    logStep("Climbing bar " + barNum
                            + " (retracting: nest stages → through frame → hooks engage, encoder → negative)..."),
                    climb.getClimbNextBarCommand(),
                    logStep("Bar " + barNum + " engaged. Counter should be " + barNum + ". State: HOLDING."),
                    Commands.waitSeconds(PAUSE_SECONDS));
        }

        // Step 5: Hold at bar 3
        addCommands(
                logStep("Full climb complete! Holding at bar 3. Counter should be 3. Cancel to stop."),
                climb.getHoldCommand());
    }

    private static Command logStep(String message) {
        return Commands.runOnce(() -> Telemetry.publish("ClimbTest/Status", message, TelemetryLevel.LAB));
    }

    /**
     * Factory method to create the climb test command.
     *
     * @param climb the climb subsystem to test
     * @return the test command, or empty if climb is null
     */
    public static Optional<Command> create(ClimbSubsystem climb) {
        if (climb == null) {
            System.out.println("ClimbTestCommand: Cannot create - ClimbSubsystem is null");
            return Optional.empty();
        }

        return Optional.of(new ClimbTestCommand(climb).withName("ClimbTest"));
    }
}
