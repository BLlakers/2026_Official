package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.support.Telemetry;
import frc.robot.support.TelemetryLevel;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * A test command that navigates the robot to each AprilTag on the field in sequence.
 *
 * For each tag, the robot:
 * 1. Drives to a position 2 meters away from the tag
 * 2. Orients itself to face the tag
 * 3. Pauses for 1 second
 * 4. Moves to the next tag
 *
 * At the final tag, the robot performs a 360-degree spin to indicate completion,
 * then returns to the first tag and repeats the routine indefinitely.
 *
 * This command is useful for validating vision system integration in simulation.
 */
public class VisionAlignmentTestCommand extends SequentialCommandGroup {

    private static final double STANDOFF_DISTANCE_METERS = 2.0;
    private static final double PAUSE_DURATION_SECONDS = 1.0;

    // Path constraints for navigation - fast movement
    private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
            3.5, // Max velocity (m/s)
            2.5, // Max acceleration (m/s²)
            1.5 * Math.PI, // Max angular velocity (rad/s)
            Math.PI // Max angular acceleration (rad/s²)
            );

    private final List<Pose2d> targetPoses;
    private int loopCount = 0;

    /**
     * Private constructor - use the static create() factory method.
     */
    private VisionAlignmentTestCommand(List<TargetInfo> targets, Drivetrain drivetrain) {
        this.targetPoses = new ArrayList<>();

        if (targets.isEmpty()) {
            System.out.println("VisionAlignmentTestCommand: No AprilTags found in field layout!");
            return;
        }

        // Build the sequential command
        for (int i = 0; i < targets.size(); i++) {
            TargetInfo target = targets.get(i);
            boolean isLastTag = (i == targets.size() - 1);

            // Log which tag we're navigating to
            addCommands(Commands.runOnce(() -> {
                Telemetry.publish("VisionTest/CurrentTagID", target.tagId, TelemetryLevel.LAB);
                Telemetry.publish("VisionTest/Status", "Navigating to Tag " + target.tagId, TelemetryLevel.LAB);
            }));

            // Drive to the target pose (5m from tag, facing it)
            addCommands(createDriveToTagCommand(target.targetPose, target.tagId));

            // Pause
            addCommands(
                    Commands.runOnce(() -> Telemetry.publish(
                            "VisionTest/Status", "Paused at Tag " + target.tagId, TelemetryLevel.LAB)),
                    Commands.waitSeconds(PAUSE_DURATION_SECONDS));

            // If this is the last tag, do a 360-degree spin
            if (isLastTag) {
                addCommands(
                        Commands.runOnce(() ->
                                Telemetry.publish("VisionTest/Status", "Completing 360° spin", TelemetryLevel.LAB)),
                        createSpinCommand(drivetrain),
                        Commands.runOnce(() -> {
                            loopCount++;
                            Telemetry.publish("VisionTest/LoopCount", loopCount, TelemetryLevel.LAB);
                            Telemetry.publish(
                                    "VisionTest/Status",
                                    "Loop " + loopCount + " complete, restarting...",
                                    TelemetryLevel.LAB);
                        }));
            }

            targetPoses.add(target.targetPose);
        }

        // Log the planned route
        System.out.println("VisionAlignmentTestCommand: Planning route to " + targets.size() + " AprilTags");
        for (TargetInfo target : targets) {
            System.out.printf(
                    "  Tag %d: Target pose (%.2f, %.2f) facing tag at (%.2f, %.2f)%n",
                    target.tagId,
                    target.targetPose.getX(),
                    target.targetPose.getY(),
                    target.tagPose.getX(),
                    target.tagPose.getY());
        }
    }

    /**
     * Calculates a pose that is a specified distance away from the tag,
     * with the robot facing toward the tag.
     *
     * The standoff position is calculated by moving in the direction the tag is facing
     * (i.e., in front of the tag from the tag's perspective).
     */
    private static Pose2d calculateStandoffPose(Pose2d tagPose, double distance) {
        // The tag's rotation points outward from the surface it's mounted on
        // We want to position the robot in front of the tag
        Rotation2d tagRotation = tagPose.getRotation();

        // Move in the direction the tag is facing (outward from the tag)
        double offsetX = distance * tagRotation.getCos();
        double offsetY = distance * tagRotation.getSin();

        Translation2d standoffPosition = new Translation2d(tagPose.getX() + offsetX, tagPose.getY() + offsetY);

        // Robot should face back toward the tag (opposite of the tag's facing direction)
        Rotation2d robotRotation = tagRotation.plus(Rotation2d.fromDegrees(180));

        return new Pose2d(standoffPosition, robotRotation);
    }

    /**
     * Checks if a pose is within reasonable field bounds.
     * FRC field is approximately 16.5m x 8.0m.
     */
    private static boolean isWithinFieldBounds(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();

        // Allow some margin inside the field walls
        double margin = 0.5; // 0.5m margin from walls
        return x >= margin && x <= 16.5 - margin && y >= margin && y <= 8.0 - margin;
    }

    /**
     * Creates a command to drive to a specific pose using PathPlanner's pathfinding.
     */
    private Command createDriveToTagCommand(Pose2d targetPose, int tagId) {
        return Commands.either(
                // If AutoBuilder is configured, use pathfinding
                AutoBuilder.pathfindToPose(targetPose, PATH_CONSTRAINTS, 0.0),
                // Otherwise, log an error
                Commands.runOnce(() -> System.out.println(
                        "VisionAlignmentTestCommand: AutoBuilder not configured, cannot navigate to Tag " + tagId)),
                AutoBuilder::isConfigured);
    }

    /**
     * Creates a command that spins the robot 360 degrees.
     * Uses a simple timed rotation approach.
     */
    private static Command createSpinCommand(Drivetrain drivetrain) {
        // Spin at a controlled angular velocity for a full rotation
        double spinVelocity = 1.5 * Math.PI; // rad/s (270 deg/s) - faster spin
        double spinDuration = 2.0 * Math.PI / spinVelocity; // Time for 360 degrees

        return Commands.run(() -> drivetrain.drive(0, 0, spinVelocity), drivetrain)
                .withTimeout(spinDuration)
                .andThen(Commands.runOnce(drivetrain::stopModules, drivetrain));
    }

    /**
     * Gets the list of target poses this command will visit.
     * Useful for visualization or debugging.
     */
    public List<Pose2d> getTargetPoses() {
        return new ArrayList<>(targetPoses);
    }

    /**
     * Factory method to create the command with safety checks.
     * The command will loop indefinitely until cancelled.
     */
    public static Optional<Command> create(Drivetrain drivetrain) {
        if (!AutoBuilder.isConfigured()) {
            System.out.println("VisionAlignmentTestCommand: Cannot create - AutoBuilder not configured");
            return Optional.empty();
        }

        // Load field layout and build target list
        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        List<TargetInfo> targets = buildTargetList(fieldLayout);

        if (targets.isEmpty()) {
            System.out.println("VisionAlignmentTestCommand: No valid AprilTag targets found");
            return Optional.empty();
        }

        // Create a command that loops forever by using repeatedly()
        // Each iteration visits all tags, spins, then starts over
        Command loopingCommand = Commands.defer(
                        () -> new VisionAlignmentTestCommand(targets, drivetrain), Set.of(drivetrain))
                .repeatedly();

        return Optional.of(loopingCommand.withName("VisionAlignmentTest"));
    }

    /**
     * Builds a list of target poses from the field layout.
     * Tags are sorted by ID for consistent ordering.
     */
    private static List<TargetInfo> buildTargetList(AprilTagFieldLayout fieldLayout) {
        List<TargetInfo> targets = new ArrayList<>();

        fieldLayout.getTags().stream()
                .sorted(Comparator.comparingInt(tag -> tag.ID))
                .forEach(tag -> {
                    Pose3d tagPose3d = tag.pose;
                    Pose2d tagPose = tagPose3d.toPose2d();

                    // Calculate the target pose: 5m away from the tag, facing it
                    Pose2d targetPose = calculateStandoffPose(tagPose, STANDOFF_DISTANCE_METERS);

                    // Only add if the target pose is within reasonable field bounds
                    if (isWithinFieldBounds(targetPose)) {
                        targets.add(new TargetInfo(tag.ID, tagPose, targetPose));
                    } else {
                        System.out.printf(
                                "VisionAlignmentTestCommand: Skipping Tag %d - standoff position out of bounds%n",
                                tag.ID);
                    }
                });

        return targets;
    }

    /**
     * Internal record to hold tag and target pose information.
     */
    private record TargetInfo(int tagId, Pose2d tagPose, Pose2d targetPose) {}
}
