package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.support.DIOChannel;
import java.util.Arrays;
import java.util.List;

public final class Constants {

    // TODO: Find a better home
    public static final double MAX_DRIVE_MOTOR_SPEED = 14.25;

    // TODO: Find a better home
    public static final double MAX_TURN_MOTOR_SPEED = 12.5;

    /**
     * Feature flags to enable/disable subsystems during incremental robot bring-up.
     * Flip these to
     * true as hardware becomes available on the robot.
     */
    public static final class FeatureFlags {
        public static final boolean ENABLE_LED_STRAND = false;
        public static final boolean ENABLE_VISION = false;

        public static final boolean ENABLE_INTAKE = true;
        public static final boolean ENABLE_RELAY = true;
        public static final boolean ENABLE_INDEXER = true;
        public static final boolean ENABLE_SHOOTER = true;
    }

    public static final class DriverLabels {
        public static final String ASA = "Asa";
        public static final String BEN = "Ben";
    }

    public static class Drive {
        // (x, y) position of each module relative to the robot center (center of rotation)
        public static final Translation2d SMFrontRightLocation = new Translation2d(0.2533, -0.2533);
        public static final Translation2d SMFrontLeftLocation = new Translation2d(0.2533, 0.2533);
        public static final Translation2d SMBackLeftLocation = new Translation2d(-0.2533, 0.2533);
        public static final Translation2d SMBackRightLocation = new Translation2d(-0.2533, -0.2533);
    }

    public static class Conversion {
        public static final double kWheelDiameterM = Inches.of(4).in(Meters);
        public static final double wheelRadius = kWheelDiameterM / 2.0;
        public static final double kWheelCircumference = Math.PI * kWheelDiameterM;

        // TODO: Remove these legacy values one once drivetrain is calibrated and validated/calibrated
        @Deprecated
        public static final double NeoMaxSpeedRPM = 5820;

        // TODO: Remove these legacy values one once drivetrain is calibrated and validated/calibrated
        @Deprecated
        public static final double TurnGearRatio = 12.8;

        public static final double DriveGearRatio = 8.14;
    }

    public static class Controller {
        public static final int DRIVER_CONTROLLER_CHANNEL = 0;
        public static final int MANIPULATION_CONTROLLER_CHANNEL = 1;
        public static final int DEBUG_CONTROLLER_CHANNEL = 2;
        public static final double deadzone = 0.17;
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
    }

    public static class Port {
        public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
        public static final int FRONT_LEFT_TURN_CHANNEL = 2;

        public static final int FRONT_RIGHT_DRIVE_CHANNEL = 3;
        public static final int FRONT_RIGHT_TURN_CHANNEL = 4;

        public static final int BACK_RIGHT_DRIVE_CHANNEL = 5;
        public static final int BACK_RIGHT_TURN_CHANNEL = 6;

        public static final int BACK_LEFT_DRIVE_CHANNEL = 7;
        public static final int BACK_LEFT_TURN_CHANNEL = 8;

        public static final int FRONT_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ZERO.getChannel();
        public static final int FRONT_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ONE.getChannel();
        public static final int BACK_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.TWO.getChannel();
        public static final int BACK_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.THREE.getChannel();

        // DIO 4–9: Available.
    }

    /**
     * Constants for the Intake subsystem.
     *
     * <p>The intake collects fuel balls via a roller chain and captures them in an on-robot fabric
     * bag. A 2-motor lift (25:1 NEO) raises or lowers the entire intake assembly to satisfy the
     * frame-perimeter size rule. A single NEO Vortex (1:1) drives the intake rollers.
     *
     * <p>Encoder zero = intake fully lowered (bumper contact). Positive = intake raised.
     * All speeds and setpoints are TODO until physical testing.
     */
    public static class IntakeConstants {

        // -------------------------------------------------------------------------
        // CAN IDs — confirm with build team before first power-on
        // -------------------------------------------------------------------------

        /** CAN ID for the roller NEO Vortex (SparkFlex). */
        public static final int ROLLER_MOTOR_ID = 9;

        /** CAN ID for lift motor 1 (NEO / SparkMax — right side). */
        public static final int LIFT_MOTOR_1_ID = 10;

        /** CAN ID for lift motor 2 (NEO / SparkMax — left side, likely inverted). */
        public static final int LIFT_MOTOR_2_ID = 11;

        // -------------------------------------------------------------------------
        // Current limits
        // -------------------------------------------------------------------------

        public static final int ROLLER_CURRENT_LIMIT = 40; // amps
        public static final int LIFT_CURRENT_LIMIT = 40; // amps

        // -------------------------------------------------------------------------
        // Gear ratio
        // -------------------------------------------------------------------------

        /** Gear reduction between NEO shaft and lift output. 25:1. */
        public static final double LIFT_GEAR_RATIO = 25.0;

        // -------------------------------------------------------------------------
        // Roller speeds [-1.0, 1.0]
        // -------------------------------------------------------------------------

        /** Intake speed — rollers spin inward to collect balls. Positive. */
        public static final double INTAKE_SPEED = 0.7; // TDO: tune

        /** Reverse speed — rollers spin outward to eject. Negative. */
        public static final double REVERSE_SPEED = -0.6; // TODO: tune

        // -------------------------------------------------------------------------
        // Lift speeds [-1.0, 1.0]
        // Convention: positive = raise intake, negative = lower intake
        // -------------------------------------------------------------------------

        /** Speed for raising intake to stowed position. Positive. */
        public static final double RAISE_SPEED = 0.25; // TODO: tune

        /**
         * Speed for lowering intake to match position.
         * Negative; kept slower than raise since gravity assists.
         */
        public static final double LOWER_SPEED = -0.2; // TODO: tune

        /**
         * Speed for homing — slow upward creep until the retracted hardstop is detected via current.
         * Positive. Kept slow to avoid hard impact.
         */
        public static final double HOMING_SPEED = 0.15; // TODO: tune

        // -------------------------------------------------------------------------
        // Homing — retracted-hardstop current detection
        // -------------------------------------------------------------------------

        /**
         * Current threshold (amps) that signals the intake has reached the retracted hardstop.
         * Homing stops when EITHER lift motor exceeds this threshold.
         *
         * <p>Tune empirically: run homing, watch
         * {@code Intake/Lift/Motor1/Current} and {@code Intake/Lift/Motor2/Current},
         * note the spike when the assembly contacts the retracted hardstop, then set just below it.
         */
        public static final double HOMING_CURRENT_THRESHOLD_AMPS = 20.0; // TODO: tune empirically

        // -------------------------------------------------------------------------
        // Encoder setpoints (lift motor rotations from homed zero)
        // Zero = fully retracted (hardstop contact). Negative = intake lowered.
        // -------------------------------------------------------------------------

        /** Encoder position at fully-lowered (match) position. Negative from homed zero. */
        public static final double LOWERED_POSITION_ROTATIONS = -50.0; // TODO: measure empirically

        /** Encoder position at fully-retracted (stowed) position. Established by homing. */
        public static final double RAISED_POSITION_ROTATIONS = 1.0;

        /**
         * Acceptable position error (rotations) for setpoint commands.
         * TODO: tighten after physical testing.
         */
        public static final double POSITION_TOLERANCE_ROTATIONS = 0.2;

        // -------------------------------------------------------------------------
        // ProfiledPIDController gains (duty-cycle output per rotation of tracking error)
        // Start conservative — the profile does most of the work; PID corrects small
        // deviations from the trapezoidal trajectory.
        // -------------------------------------------------------------------------

        /** Proportional gain. At 10-rotation tracking error → 0.5 duty cycle output. TODO: tune. */
        public static final double LIFT_PID_KP = 0.05;
        /** Integral gain. Start at 0; add only if mechanism consistently undershoots. */
        public static final double LIFT_PID_KI = 0.0;
        /** Derivative gain. Add only if mechanism oscillates at the setpoint. TODO: tune. */
        public static final double LIFT_PID_KD = 0.005;

        // -------------------------------------------------------------------------
        // TrapezoidProfile motion constraints (motor rotations / second)
        // Total travel ≈ 50 rotations. At 20 rot/s max, the traverse takes ~2.5 s.
        // -------------------------------------------------------------------------

        /**
         * Maximum profiled velocity in motor rotations per second.
         * 20 rot/s ≈ 480 RPM through 25:1 (conservative). Increase if motion is too slow.
         * TODO: tune on robot.
         */
        public static final double LIFT_MAX_VELOCITY_ROTS_PER_SEC = 20.0;

        /**
         * Maximum profiled acceleration in motor rotations per second squared.
         * 10 rot/s² → 2-second ramp to max velocity. Reduce if start/stop is too abrupt.
         * TODO: tune on robot.
         */
        public static final double LIFT_MAX_ACCEL_ROTS_PER_SEC_SQ = 10.0;

        // -------------------------------------------------------------------------
        // Extended (lower) hardstop current detection
        // Mirrors the retracted hardstop detection used during homing.
        // -------------------------------------------------------------------------

        /**
         * Current threshold (amps) signalling extended (lower) hardstop contact.
         * getLowerCommand() terminates when EITHER motor exceeds this value.
         *
         * <p>Tune empirically: run getLowerCommand(), watch
         * {@code Intake/Lift/Motor1/Current} and {@code Intake/Lift/Motor2/Current},
         * note the spike when the intake contacts the lower hardstop, then set just below it.
         */
        public static final double EXTENDED_HARDSTOP_CURRENT_THRESHOLD_AMPS = 20.0; // TODO: tune
    }

    public static class RelayConstants {

        // -------------------------------------------------------------------------
        // CAN IDs — confirm with build team before first power-on
        // -------------------------------------------------------------------------

        /** CAN ID for the relay drive motor (NEO on SparkMax). */
        public static final int RELAY_MOTOR_ID = 13;

        // -------------------------------------------------------------------------
        // Current limits
        // -------------------------------------------------------------------------

        public static final int RELAY_CURRENT_LIMIT = 40; // amps

        // -------------------------------------------------------------------------
        // Gear ratio
        // -------------------------------------------------------------------------

        /** Gear reduction between NEO shaft and innermost roller bar. 10:1. */
        public static final double RELAY_GEAR_RATIO = 10.0;

        // -------------------------------------------------------------------------
        // Speeds [-1.0, 1.0]
        // Convention: positive = convey balls toward indexer
        // -------------------------------------------------------------------------

        /** Speed for conveying balls toward the indexer. Positive. */
        public static final double RELAY_SPEED = 0.15;

        /** Speed for reversing to clear jams. Negative. */
        public static final double RELAY_REVERSE_SPEED = -0.15;
    }

    public static class IndexerConstants {

        // -------------------------------------------------------------------------
        // CAN IDs — confirm with build team before first power-on
        // -------------------------------------------------------------------------

        /** CAN ID for the indexer drive motor (NEO on SparkMax). */
        public static final int INDEXER_MOTOR_ID = 14;

        // -------------------------------------------------------------------------
        // Current limits
        // -------------------------------------------------------------------------

        public static final int INDEXER_CURRENT_LIMIT = 40; // amps

        // -------------------------------------------------------------------------
        // Gear ratio
        // -------------------------------------------------------------------------

        /** Direct drive — NEO shaft to driveshaft. 1:1. */
        public static final double INDEXER_GEAR_RATIO = 1.0;

        // -------------------------------------------------------------------------
        // Speeds [-1.0, 1.0]
        // Convention: positive = pull balls into chamber and advance toward shooter
        // -------------------------------------------------------------------------

        /** Speed for indexing balls toward the . Positive. */
        public static final double INDEXER_SPEED = 0.65;

        /** Speed for reversing to clear jams. Negative. */
        public static final double INDEXER_REVERSE_SPEED = -0.47;
    }

    /**
     * Constants for the Shooter subsystem.
     *
     * <p>Single-flywheel shooter. Speed convention: positive output fires the ball toward
     * the target. All speed values are open-loop percent output placeholders until
     * calibrated RPM targets are established.
     */
    public static class ShooterConstants {

        // -------------------------------------------------------------------------
        // CAN IDs — confirm with build team before first power-on
        // -------------------------------------------------------------------------

        /** CAN ID for the shooter flywheel motor (NEO on SparkMax). */
        public static final int SHOOTER_MOTOR_ID = 16;

        // -------------------------------------------------------------------------
        // Current limits
        // -------------------------------------------------------------------------

        public static final int SHOOTER_CURRENT_LIMIT = 40; // amps

        // -------------------------------------------------------------------------
        // Gear ratios — direct-drive (motor shaft = flywheel axle)
        // -------------------------------------------------------------------------

        /** Flywheel gear ratio — direct drive, NEO shaft to flywheel. 1:1. */
        public static final double SHOOTER_GEAR_RATIO = 1.0;

        // -------------------------------------------------------------------------
        // Open-loop speeds [-1.0, 1.0]
        // Convention: positive = ball fired toward target
        // These are stubs — replace with physics-solver RPM targets after calibration.
        // -------------------------------------------------------------------------

        /**
         * Open-loop speed for the flywheel when shooting. Positive.
         * <b>TODO: replace with physics-solver RPM target after calibration sessions.</b>
         */
        public static final double SHOOTER_SPEED = 1.0; // TODO: tune

        /**
         * Open-loop speed for the flywheel when reversing. Negative.
         * <b>TODO: tune for effective jam clearing.</b>
         */
        public static final double SHOOTER_REVERSE_SPEED = -0.5; // TODO: tune

        // ---------------------------------------------------------------------
        // Feedforward & PID defaults for closed-loop RPM control (tuning required)
        // ---------------------------------------------------------------------

        /** Static gain (volts) to overcome stiction / breakaway. Measured during tuning. */
        public static final double SHOOTER_KS = 0.0;

        /** Velocity gain (volts per rad/s) — measured from V vs omega data. */
        public static final double SHOOTER_KV = 0.0180;

        /** Acceleration gain (volts per rad/s^2) — optional for aggressive control. */
        public static final double SHOOTER_KA = 0.0;

        // PID gains (units: volts per RPM for P, etc.). Start at zero and tune on robot.
        public static final double SHOOTER_kP = 0.0022;
        public static final double SHOOTER_kI = 0.0015;
        public static final double SHOOTER_kD = 0.0;

        // Mapping from Limelight-measured distance (meters) to RPM: RPM = OFFSET + PER_METER * distance
        // These defaults are placeholders; tune on the robot or provide a lookup table for accuracy.
        public static final double SHOOTER_ADVANCE_RPM = 3250;
        public static final double SHOOTER_RPM_OFFSET = 2000.0; // RPM at zero distance (placeholder)
        public static final double SHOOTER_RPM_PER_METER = 300.0; // additional RPM per meter (placeholder)
    }

    public class TurnEncoderOffsets {
        public static final double flTurnEncoderOffset = 0.741;
        public static final double frTurnEncoderOffset = 1.697;
        public static final double blTurnEncoderOffset = -0.100;
        public static final double brTurnEncoderOffset = 3.790;
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
}
