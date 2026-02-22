package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.*;
import frc.robot.support.DIOChannel;
import frc.robot.support.RobotVersion;
import java.util.Arrays;
import java.util.List;

public final class Constants {

    /**
     * Feature flags to enable/disable subsystems during incremental robot bring-up.
     * Flip these to true as hardware becomes available on the robot.
     */
    public static final class FeatureFlags {
        public static final boolean ENABLE_FUEL = false;
        public static final boolean ENABLE_TURRET_TRACKER = true;
        public static final boolean ENABLE_LED_STRAND = false;

        // These stay enabled for initial drivetrain testing
        public static final boolean ENABLE_DRIVETRAIN = true;
        public static final boolean ENABLE_VISION = true;

        public static final boolean ENABLE_CLIMB = true;
    }

    public static final class DriverLabels {
        public static final String ASA = "Asa";
        public static final String BEN = "Ben";
    }

    public static class Drive {
        // (x, y) position of each module relative to the robot center (center of rotation)
        public static final Translation2d SMFrontRightLocation = new Translation2d(0.285, -0.285);
        public static final Translation2d SMFrontLeftLocation = new Translation2d(0.285, 0.285);
        public static final Translation2d SMBackLeftLocation = new Translation2d(-0.285, 0.285);
        public static final Translation2d SMBackRightLocation = new Translation2d(-0.285, -0.285);

        // Camera-to-robot transform (now configured in VisionSubsystemContext)
        public static final Transform3d CAMERA_TO_ROBOT = new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0));
    }

    public static class Conversion {
        public static final double driveEncoderCtsperRev = 6.8;
        public static final double kWheelDiameterM = Inches.of(4).in(Meters);
        public static final double wheelRadius = kWheelDiameterM / 2.0;
        public static final double kWheelCircumference = Math.PI * kWheelDiameterM;
        public static final double NeoEncoderCountsPerRev = 42;
        public static final double NeoRevPerEncoderCounts = 1 / NeoEncoderCountsPerRev;
        public static final double NeoMaxSpeedRPM = 5820;
        public static final double MagEncoderCountsPerRev = 4096;
        public static final double MagRevPerEncoderCounts = 1 / MagEncoderCountsPerRev;
        public static final double DriveGearRatio = 8.14;
        public static final double TurnGearRatio = 12.8;
        public static final double driveEncoderConversion = DriveGearRatio * kWheelCircumference;
    }

    public static class Controller {
        public static final int DRIVER_CONTROLLER_CHANNEL = 0;
        public static final int MANIPULATION_CONTROLLER_CHANNEL = 1;
        public static final int DEBUG_CONTROLLER_CHANNEL = 2;
        public static final int buttonA = 1;
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        public static final int buttonLeft = 5;
        public static final int buttonRight = 6;
        public static final int buttonOptions = 7;
        public static final int buttonStart = 8;
        public static final int buttonLS = 9;
        public static final int buttonRS = 10;
        public static final double deadzone = 0.17;
        public static final double RTdeadzone = .01;
    }

    public static class AprilTagID {
        public static final int PracticeSpeakerCenter = 1;
        public static final int BlueSpeakerCenter = 7;
        public static final int RedSpeakerCenter = 4;

        public static final int BlueStageCenter = 14;
        public static final int RedStageCenter = 13;

        public static final int BlueStageLeft = 15;
        public static final int RedStageRight = 12;

        public static final int RedStageLeft = 11;
        public static final int BlueStageRight = 16;

        public static final Pose2d BlueSpeakerCenterPose = new Pose2d(); // TODO
        public static final Pose2d RedSpeakerCenterPose = new Pose2d(); // TODO
    }

    /**
     * Hub AprilTag face definitions for the 2026 game.
     * Each hub has 4 faces (N/S/E/W), each with 2 tags at Z=1.124m.
     * Tag pairs per face and their approximate face-normal directions.
     */
    public static class Hub {
        // Blue hub face tag pairs (tags at Z=1.124m only; corners 17,22,23,28 excluded)
        public static final int[] BLUE_WEST_TAGS = {25, 26}; // Facing -X (toward blue wall)
        public static final int[] BLUE_EAST_TAGS = {19, 20}; // Facing +X (toward red wall)
        public static final int[] BLUE_NORTH_TAGS = {21, 24}; // Facing +Y
        public static final int[] BLUE_SOUTH_TAGS = {18, 27}; // Facing -Y

        // Red hub face tag pairs (tags at Z=1.124m only; corners 1,6,7,12 excluded)
        public static final int[] RED_WEST_TAGS = {3, 4}; // Facing -X (toward blue wall)
        public static final int[] RED_EAST_TAGS = {9, 10}; // Facing +X (toward red wall)
        public static final int[] RED_NORTH_TAGS = {2, 11}; // Facing +Y
        public static final int[] RED_SOUTH_TAGS = {5, 8}; // Facing -Y

        // All blue hub face tags (for iteration)
        public static final int[][] BLUE_FACES = {BLUE_WEST_TAGS, BLUE_EAST_TAGS, BLUE_NORTH_TAGS, BLUE_SOUTH_TAGS};
        public static final int[][] RED_FACES = {RED_WEST_TAGS, RED_EAST_TAGS, RED_NORTH_TAGS, RED_SOUTH_TAGS};

        public static final double DEFAULT_TURRET_RANGE_DEGREES = 270.0;
    }

    public static class Port {
        public static final int REAR_LEFT_TURN_CHANNEL = 2;
        public static final int REAR_LEFT_DRIVE_CHANNEL = 1;
        public static final int FRONT_LEFT_DRIVE_CHANNEL = 7;
        public static final int FRONT_LEFT_STEER_CHANNEL = 3;
        public static final int FRONT_RIGHT_STEER_CHANNEL = 6;
        public static final int FRONT_RIGHT_DRIVE_CHANNEL = 4;
        public static final int REAR_RIGHT_DRIVE_CHANNEL = 5;
        public static final int REAR_RIGHT_STEER_CHANNEL = 8;
        public static final int ELEVATOR_DRIVE_CHANNEL = 11;
        public static final int CLIMB_DRIVE_CHANNEL = 12;
        public static final int ELEVATOR_FOLLOWER_DRIVE_CHANNEL = 15;
        public static final int FRONT_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ZERO.getChannel();
        public static final int FRONT_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ONE.getChannel();
        public static final int REAR_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.TWO.getChannel();
        public static final int REAR_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.THREE.getChannel();
        public static final int climbMagSwitchDIOC = DIOChannel.FOUR.getChannel();
        public static final int hangerLeftMagSwitchDIOC = DIOChannel.SEVEN.getChannel();
        public static final int hangerRightMagSwitchDIOC = DIOChannel.EIGHT.getChannel();
        public static final int PHChannel = 30; // REV Pneumatic Hub
        public static final int PDHChannel = 20; // REV Power Distribution Hub
    }

    public static class Algae {
        public static final int intakeMotorChannel = 9;
        public static final int m_AlgaeMtrC = 10;
    }

    public static class ClimbConstants {
        // Motor CAN ID — matches Port.CLIMB_DRIVE_CHANNEL
        public static final int MOTOR_ID = Port.CLIMB_DRIVE_CHANNEL; // 12

        // Mechanism geometry — confirm from CAD / physical measurement
        /** Gear ratio between motor shaft and spool. Motor rotations = spool rotations × gearRatio. */
        public static final double GEAR_RATIO = 5.0; // TODO: 25.0

        /** Circumference of the cord spool in meters (π × spool diameter). */
        public static final double SPOOL_CIRCUMFERENCE_METERS = 0.0635; // 2.5"

        // Motor output speeds [-1.0, 1.0]
        // Convention: positive = telescope extends UP, negative = telescope retracts (toward stored / through frame)
        /** Speed for extending telescope upward (reaching for bar). Should be positive. */
        public static final double EXTEND_UP_SPEED = 1.0; // TODO: tune

        /** Speed for retracting telescope (nesting stages / pulling through frame). Should be negative. */
        public static final double RETRACT_SPEED = -1.0; // TODO: tune

        /**
         * Slow speed for homing (retracts telescope toward stored/ground hardstop).
         * Kept slower than RETRACT_SPEED to avoid excessive ground impact.
         * Robot MUST be on the ground for homing — the ground provides the hardstop.
         */
        public static final double HOMING_SPEED = -0.15; // TODO: tune

        // Current limits
        public static final int MOTOR_CURRENT_LIMIT = 40; // amps

        /**
         * Current threshold (amps) that signals the telescope has reached the ground-contact
         * hardstop during homing. There is no internal mechanical hardstop between stages — the
         * current spike occurs when the stages are nested and the ground prevents the assembly
         * from traveling further through the frame.
         *
         * <p>Tune empirically: run a slow homing routine on a flat surface, watch
         * {@code Climb/Motor/Current} in Shuffleboard, note the spike when the assembly
         * bottoms out against the ground, then set this just below it.
         */
        public static final double HOMING_CURRENT_THRESHOLD_AMPS = 15.0; // TODO: tune empirically

        // -------------------------------------------------------------------------
        // Encoder setpoints — motor rotations from zero (= stored / nested / on ground)
        //
        // Encoder convention:
        //   0        = stored (stages nested, assembly at lowest frame position, on ground)
        //   positive = second stage extended upward (reaching for bar)
        //   negative = assembly traveled through frame bottom (hooks rising toward bar)
        //
        // The hardstop at zero only works on the ground. When hanging, the motor
        // overcomes the first-stage spring and the encoder freely goes negative.
        // -------------------------------------------------------------------------

        /** Encoder position at stored/nested state on the ground. Encoder is zeroed here after homing. */
        public static final double STORED_POSITION_ROTATIONS = 0.0;

        // --- Auto setpoints ---

        /**
         * Encoder position (positive) to extend to reach bar 1 during auto.
         * Same as BAR_1_EXTEND_ROTATIONS — the top hook must reach bar 1.
         * TODO: measure empirically.
         */
        public static final double AUTO_EXTEND_ROTATIONS = 50.0;

        /**
         * Encoder position for the auto lift — just enough retraction to lift the robot off the
         * ground. This is a partial retraction (still positive or slightly negative) — hooks do
         * NOT need to engage. Followed by getLowerToGroundCommand() at teleop start.
         * TODO: measure empirically.
         */
        public static final double AUTO_ENGAGE_ROTATIONS = 20.0;

        // --- Teleop per-bar setpoints ---
        // Each bar requires two setpoints: extend (reach the bar) and engage (retract until
        // passive hooks catch). Extend values are positive; engage values are negative.
        // Bar 1 extend is largest because the ground-to-bar-1 distance > bar-to-bar distance.

        /** Encoder position (positive) — extend UP to reach bar 1 from ground. Longest reach. */
        public static final double BAR_1_EXTEND_ROTATIONS = 50.0; // TODO: measure

        /** Encoder position (negative) — retract through frame until hooks engage bar 1. */
        public static final double BAR_1_ENGAGE_ROTATIONS = -20.5; // TODO: measure

        /** Encoder position (positive) — extend UP to reach bar 2 from bar 1. Shorter than bar 1. */
        public static final double BAR_2_EXTEND_ROTATIONS = 35.0; // TODO: measure

        /** Encoder position (negative) — retract through frame until hooks engage bar 2. */
        public static final double BAR_2_ENGAGE_ROTATIONS = -20.5; // TODO: measure

        /** Encoder position (positive) — extend UP to reach bar 3 from bar 2. Similar to bar 2. */
        public static final double BAR_3_EXTEND_ROTATIONS = 33.0; // TODO: measure

        /** Encoder position (negative) — retract through frame until hooks engage bar 3. */
        public static final double BAR_3_ENGAGE_ROTATIONS = -20.5; // TODO: measure

        /**
         * Acceptable position error (rotations) when checking if a setpoint has been reached.
         * Larger values complete commands sooner; smaller values are more precise.
         * TODO: tune — start at 1.0 and tighten if position isn't accurate enough.
         */
        public static final double POSITION_TOLERANCE_ROTATIONS = 1.0;

        // -------------------------------------------------------------------------
        // AdvantageScope Pose3d visualization constants
        // -------------------------------------------------------------------------

        /**
         * Telescope visual length (meters) at full upward extension.
         * Based on 15" lower stage + 13.25" upper stage = 28.25" = 0.718 m.
         */
        public static final double MAX_TELESCOPE_LENGTH = 0.718;

        /**
         * Minimum visual telescope arm length (meters), used in both the stored position and the
         * through-frame retraction regime.
         *
         * <p>Derived so that the telescope tip reaches {@code HOOK_MOUNT_HEIGHT_METERS} exactly when
         * the encoder is at {@code BAR_1_ENGAGE_ROTATIONS}:
         *
         * <pre>
         *   assemblyShiftAtEngage = |BAR_1_ENGAGE_ROTATIONS| × SPOOL_CIRCUMFERENCE / GEAR_RATIO
         *                         = 20.5 × 0.0635 / 5.0 = 0.26035 m
         *   MIN_TELESCOPE_LENGTH  = HOOK_MOUNT_HEIGHT − 0.05 + assemblyShiftAtEngage
         *                         = 0.2667 − 0.05 + 0.26035 = 0.4771 m
         * </pre>
         *
         * <p>Note: at encoder = 0 (stored), the telescope top renders at 0.05 + 0.4771 ≈ 20.75",
         * which is an acceptable visualization approximation. This constant does not affect motor
         * behaviour.
         */
        public static final double MIN_TELESCOPE_LENGTH = 0.4771;

        /** Horizontal portion of the passive side hook L-shape (meters). */
        public static final double SIDE_HOOK_HORIZONTAL_LENGTH = 0.100;

        /**
         * Height of the passive hook tip above the ground (meters) when the robot is on the floor.
         * 10.5 inches = 10.5 × 0.0254 = 0.2667 m.
         * At full engage (encoder = BAR_1_ENGAGE_ROTATIONS) the telescope tip descends to this
         * same height, aligning all three hooks visually.
         */
        public static final double HOOK_MOUNT_HEIGHT_METERS = 0.2667;

        /**
         * Lateral distance (meters) from robot center to each passive hook for visualization.
         * TODO: measure from CAD / physical robot.
         */
        public static final double HOOK_OFFSET_METERS = 0.1000;

        /**
         * Lateral distance (meters) from the robot center to the telescope arm.
         * Positive = left in robot frame (robot approaches the tower left-side-first).
         * TODO: measure from CAD / physical robot.
         */
        public static final double TELESCOPE_SIDE_OFFSET_METERS = 0.340;
    }

    public static class FuelConstants {
        // Motor CAN IDs
        public static final int FEEDER_MOTOR_ID = 13;
        public static final int INTAKE_LAUNCHER_MOTOR_ID = 14;

        // Voltage values for different operations
        public static final double INTAKING_FEEDER_VOLTAGE = 6.0;
        public static final double INTAKING_INTAKE_VOLTAGE = 6.0;
        public static final double LAUNCHING_FEEDER_VOLTAGE = 12.0;
        public static final double LAUNCHING_LAUNCHER_VOLTAGE = 12.0;
        public static final double SPIN_UP_FEEDER_VOLTAGE = -3.0;

        // Current limits (amps)
        public static final int FEEDER_MOTOR_CURRENT_LIMIT = 30;
        public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 40;
    }

    public abstract class RobotVersionConstants {
        public static final double flTurnEncoderOffset = 0;
        public static final double frTurnEncoderOffset = 0;
        public static final double blTurnEncoderOffset = 0;
        public static final double brTurnEncoderOffset = 0;
    }

    public class RobotVersion2025 extends RobotVersionConstants {
        public static final double flTurnEncoderOffset = 3.84 - .04 + Math.PI;
        public static final double frTurnEncoderOffset = 1.7 + Math.PI - .03 + Math.PI;
        public static final double rlTurnEncoderOffset = 3.284 + Math.PI;
        public static final double rrTurnEncoderOffset = 4.49 + Math.PI;
    }

    public class RobotVersion2023 extends RobotVersionConstants {
        public static final double flTurnEncoderOffset = 5.3038;
        public static final double frTurnEncoderOffset = Math.PI / 2 - 0.1242 - .05759;
        public static final double rlTurnEncoderOffset = 4.2 + 0.0385;
        public static final double rrTurnEncoderOffset = 2.736 - .06098;
    }

    public class RobotVersion2026 extends RobotVersionConstants {
        public static final double flTurnEncoderOffset = 3.827;
        public static final double frTurnEncoderOffset = 5.978;
        public static final double rlTurnEncoderOffset = 1.709;
        public static final double rrTurnEncoderOffset = 4.864;
    }

    public static final class Poses {
        public static final Pose2d SeventeenLeft = new Pose2d(3.824, 2.904, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d SeventeenRight = new Pose2d(4.19, 2.78, new Rotation2d(Math.toRadians(60)));
        public static final Pose2d EighteenLeft = new Pose2d(3.22, 4.06, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d EighteenRight = new Pose2d(3.22, 3.76, new Rotation2d(Math.toRadians(0)));
        public static final Pose2d NineteenLeft = new Pose2d(3.9, 5.15, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d NineteenRight = new Pose2d(3.73, 4.92, new Rotation2d(Math.toRadians(-60)));
        public static final Pose2d TwentyLeft = new Pose2d(5.18, 5.11, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d TwentyRight = new Pose2d(4.82, 5.23, new Rotation2d(Math.toRadians(-120)));
        public static final Pose2d TwentyOneLeft = new Pose2d(5.77, 3.98, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d TwentyOneRight = new Pose2d(5.78, 4.36, new Rotation2d(Math.toRadians(180)));
        public static final Pose2d TwentyTwoLeft =
                new Pose2d(5.13 /* adding 9 here */, 2.9, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d TwentyTwoRight = new Pose2d(5.34, 3.13, new Rotation2d(Math.toRadians(120)));
        public static final Pose2d SixLeft = FlippingUtil.flipFieldPose(TwentyTwoLeft);
        public static final Pose2d SixRightChanged = new Pose2d(13.94, 3.08, new Rotation2d(120));
        public static final Pose2d TwentyTwoRightChanged = FlippingUtil.flipFieldPose(SixRightChanged);
        public static final Pose2d SixRight = FlippingUtil.flipFieldPose(TwentyTwoRight);
        public static final Pose2d SevenLeft = FlippingUtil.flipFieldPose(TwentyOneLeft);
        public static final Pose2d SevenRight = FlippingUtil.flipFieldPose(TwentyOneRight);
        public static final Pose2d EightLeft = FlippingUtil.flipFieldPose(TwentyLeft);
        public static final Pose2d EightRight = FlippingUtil.flipFieldPose(TwentyRight);
        public static final Pose2d NineLeft = FlippingUtil.flipFieldPose(NineteenLeft);
        public static final Pose2d NineRight = FlippingUtil.flipFieldPose(NineteenRight);
        public static final Pose2d TenLeft = FlippingUtil.flipFieldPose(EighteenLeft);
        public static final Pose2d TenRight = FlippingUtil.flipFieldPose(EighteenRight);
        public static final Pose2d ElevenLeft = FlippingUtil.flipFieldPose(SeventeenLeft);
        public static final Pose2d ElevenRight = FlippingUtil.flipFieldPose(SeventeenRight);
        public static List<Pose2d> PositionsRed = Arrays.asList(
                SixLeft,
                SixRightChanged,
                SevenLeft,
                SevenRight,
                EightLeft,
                EightRight,
                NineLeft,
                NineRight,
                TenLeft,
                TenRight,
                ElevenLeft,
                ElevenRight,
                SeventeenLeft,
                SeventeenRight,
                EighteenLeft,
                EighteenRight,
                NineteenLeft,
                NineteenRight,
                TwentyLeft,
                TwentyRight,
                TwentyOneLeft,
                TwentyOneRight,
                TwentyTwoLeft,
                TwentyTwoRightChanged);
    }

    public static final RobotVersion defaultRobotVersion = RobotVersion.v2026;
}
