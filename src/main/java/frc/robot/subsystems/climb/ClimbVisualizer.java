package frc.robot.subsystems.climb;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import frc.robot.subsystems.drivetrain.Drivetrain;

/**
 * Publishes the climb mechanism's 3D geometry to AdvantageScope via Pose3d struct arrays.
 *
 * <p>Each component is published as a separate topic so AdvantageScope renders them as independent
 * connected polylines (same pattern as TurretTracker AimLine):
 *
 * <ul>
 *   <li>{@code Climb/Telescope} — 3 points: vertical line extending UPWARD from the robot frame,
 *       with a horizontal tip at the top (representing the top hook/catch)</li>
 *   <li>{@code Climb/HookLeft} — 3 points: L-shape for the left passive hook</li>
 *   <li>{@code Climb/HookRight} — 3 points: L-shape for the right passive hook</li>
 * </ul>
 *
 * <p>The telescope arm is mounted on the left side of the robot. Hooks are offset fore/aft of the
 * telescope along the robot's X axis and share the same lateral position.
 *
 * <p>Encoder convention: 0 = stored (nested), positive = extended upward, negative = assembly
 * through frame. The visual length tracks the positive (extension) range.
 *
 * <p>{@link #update} is a no-op when no drivetrain was provided at construction.
 */
public class ClimbVisualizer {

    private final ClimbSubsystemContext context;
    private final Drivetrain drivetrain;

    private final StructArrayPublisher<Pose3d> telescopePublisher;
    private final StructArrayPublisher<Pose3d> hookLeftPublisher;
    private final StructArrayPublisher<Pose3d> hookRightPublisher;

    /**
     * Creates a ClimbVisualizer. Pass {@code null} for {@code drivetrain} to disable publishing.
     *
     * @param context climb configuration (geometry, setpoints)
     * @param drivetrain robot drivetrain for field-relative pose; {@code null} disables publishing
     */
    public ClimbVisualizer(ClimbSubsystemContext context, Drivetrain drivetrain) {
        this.context = context;
        this.drivetrain = drivetrain;

        NetworkTableInstance nti = NetworkTableInstance.getDefault();
        this.telescopePublisher =
                nti.getStructArrayTopic("Climb/Telescope", Pose3d.struct).publish();
        this.hookLeftPublisher =
                nti.getStructArrayTopic("Climb/HookLeft", Pose3d.struct).publish();
        this.hookRightPublisher =
                nti.getStructArrayTopic("Climb/HookRight", Pose3d.struct).publish();
    }

    /**
     * Recomputes and publishes the telescope and hook Pose3d arrays based on the current encoder
     * position and robot pose.
     *
     * <p>Two visual regimes based on encoder sign:
     * <ul>
     *   <li><b>Positive encoder</b> (extending): telescope arm grows from min to max length while
     *       the base stays at the frame floor.</li>
     *   <li><b>Negative encoder</b> (through frame): telescope arm stays at min/nested length,
     *       but the entire assembly (base, arm, hooks) shifts <em>upward</em> — visually showing
     *       it sliding through the robot frame. Linear travel is computed from encoder rotations
     *       via spool circumference and gear ratio.</li>
     * </ul>
     *
     * @param encoderPosition current winch encoder position in rotations
     * @param isHomed true if the encoder has been zeroed via homing
     */
    public void update(double encoderPosition, boolean isHomed) {
        if (drivetrain == null) {
            return;
        }

        var robotPose = drivetrain.getPose2dEstimator();
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double heading = robotPose.getRotation().getRadians();

        // Fraction of upward travel: 0.0 = stored/nested, 1.0 = fully extended upward
        // Before homing, show stored (fraction = 0) since encoder is unknown.
        // Use bar 1 extend (largest) as the max range for visualization.
        double maxExtend = context.getBar1ExtendRotations();
        double fraction;
        if (!isHomed || maxExtend == 0.0) {
            fraction = 0.0;
        } else {
            // Only positive encoder values scale arm length; negative handled separately below
            fraction = Math.max(0.0, Math.min(1.0, encoderPosition / maxExtend));
        }

        // Convert robot-relative lateral offsets to field-relative coordinates.
        // Robot frame: x = forward, y = left. For a purely lateral offset dy (robot frame):
        //   dx_field = -dy * sin(heading),  dy_field = dy * cos(heading)
        double telescopeLateral = context.getTelescopeSideOffsetMeters();
        double hookOffset = context.getHookOffsetMeters();
        double hookHorizLen = context.getSideHookHorizontalLength();
        double hookTopZ = context.getHookMountHeightMeters();

        // Telescope arm base (field-relative)
        double telX = robotX + (-telescopeLateral * Math.sin(heading));
        double telY = robotY + (telescopeLateral * Math.cos(heading));
        double telTipX = telX + (-hookHorizLen * Math.sin(heading));
        double telTipY = telY + (hookHorizLen * Math.cos(heading));

        // Forward hook — offset along robot X (forward) from telescope position
        double leftX = telX + (hookOffset * Math.cos(heading));
        double leftY = telY + (hookOffset * Math.sin(heading));
        double leftTipX = leftX + (-hookHorizLen * Math.sin(heading));
        double leftTipY = leftY + (hookHorizLen * Math.cos(heading));

        // Rear hook — offset along robot X (backward) from telescope position
        double rightX = telX - (hookOffset * Math.cos(heading));
        double rightY = telY - (hookOffset * Math.sin(heading));
        double rightTipX = rightX + (-hookHorizLen * Math.sin(heading));
        double rightTipY = rightY + (hookHorizLen * Math.cos(heading));

        // Arm length: short when stored (nested), long when extended upward
        double currentArmLength = context.getMinTelescopeLength()
                + (context.getMaxTelescopeLength() - context.getMinTelescopeLength()) * fraction;

        // Through-frame shift: when encoder is negative, the entire assembly slides upward through
        // the robot frame. Convert negative encoder rotations to linear distance (meters).
        // Motor rotations → spool rotations → linear cord travel:
        //   linearTravel = |encoderRotations| × spoolCircumference / gearRatio
        double assemblyShiftZ = 0.0;
        if (isHomed && encoderPosition < 0.0 && context.getGearRatio() > 0.0) {
            assemblyShiftZ =
                    Math.abs(encoderPosition) * context.getSpoolCircumferenceMeters() / context.getGearRatio();
        }

        // Telescope base starts at a small offset above ground, shifted up when through-frame
        double telescopeBaseZ = 0.05 - assemblyShiftZ;

        // Telescope extends UPWARD — top of arm is at base + arm length
        double telescopeTopZ = telescopeBaseZ + currentArmLength;

        // Hooks are mounted on the telescope assembly — they shift upward with it
        double hookBaseZ = 0.05;
        double hookTipZ = hookTopZ;

        telescopePublisher.set(new Pose3d[] {
            new Pose3d(telX, telY, telescopeBaseZ, new Rotation3d()),
            new Pose3d(telX, telY, telescopeTopZ, new Rotation3d()),
            new Pose3d(telTipX, telTipY, telescopeTopZ, new Rotation3d())
        });

        // Left hook — L-shape: base → top of vertical → horizontal tip (outward left)
        hookLeftPublisher.set(new Pose3d[] {
            new Pose3d(leftX, leftY, hookBaseZ, new Rotation3d()),
            new Pose3d(leftX, leftY, hookTipZ, new Rotation3d()),
            new Pose3d(leftTipX, leftTipY, hookTipZ, new Rotation3d())
        });

        // Right hook — L-shape: base → top of vertical → horizontal tip (toward robot center)
        hookRightPublisher.set(new Pose3d[] {
            new Pose3d(rightX, rightY, hookBaseZ, new Rotation3d()),
            new Pose3d(rightX, rightY, hookTipZ, new Rotation3d()),
            new Pose3d(rightTipX, rightTipY, hookTipZ, new Rotation3d())
        });
    }
}
