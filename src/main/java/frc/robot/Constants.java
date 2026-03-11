package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.support.DIOChannel;

public final class Constants {

    public static final double MAX_DRIVE_MOTOR_SPEED = 14.25;
    public static final double MAX_TURN_MOTOR_SPEED = 12.5;

    public static final class FeatureFlags {
        public static final boolean ENABLE_LED_STRAND = false;
        public static final boolean ENABLE_VISION = true;
        public static final boolean ENABLE_CLIMB = false;
        public static final boolean ENABLE_INTAKE = true;
    }

    public static final class DriverLabels {
        public static final String ASA = "Asa";
    }

    public static class Drive {
        // (x, y) position of each module relative to the robot center (center of rotation)
        public static final Translation2d SMFrontRightLocation = new Translation2d(0.2533, -0.2533);
        public static final Translation2d SMFrontLeftLocation = new Translation2d(0.2533, 0.2533);
        public static final Translation2d SMBackLeftLocation = new Translation2d(-0.2533, 0.2533);
        public static final Translation2d SMBackRightLocation = new Translation2d(-0.2533, -0.2533);
    }

    public static class Conversion {
        public static final double K_WHEEL_DIAMETER_M = Inches.of(4).in(Meters);
        public static final double WHEEL_RADIUS = K_WHEEL_DIAMETER_M / 2.0;
        public static final double DRIVE_GEAR_RATIO = 8.14;
    }

    public static class Controller {
        public static final int DRIVER_CONTROLLER_CHANNEL = 0;
        public static final int MANIPULATION_CONTROLLER_CHANNEL = 1;
        public static final int DEBUG_CONTROLLER_CHANNEL = 2;
        public static final double DEADZONE = 0.17;
    }

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
        public static final int CLIMB_DRIVE_CHANNEL = 12;
        public static final int FRONT_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ZERO.getChannel();
        public static final int FRONT_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.ONE.getChannel();
        public static final int BACK_RIGHT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.TWO.getChannel();
        public static final int BACK_LEFT_TURN_ENCODER_DIO_CHANNEL = DIOChannel.THREE.getChannel();
    }

    public static class ClimbConstants {
        public static final int MOTOR_ID = Port.CLIMB_DRIVE_CHANNEL;
        public static final double GEAR_RATIO = 25.0;
        public static final double SPOOL_CIRCUMFERENCE_METERS = 0.0635;
        public static final double EXTEND_UP_SPEED = 1.0;
        public static final double RETRACT_SPEED = -1.0;
        public static final double HOMING_SPEED = -0.15;
        public static final int MOTOR_CURRENT_LIMIT = 40;
        public static final double HOMING_CURRENT_THRESHOLD_AMPS = 15.0;
        public static final double STORED_POSITION_ROTATIONS = 0.0;
        public static final double AUTO_EXTEND_ROTATIONS = 50.0;
        public static final double AUTO_ENGAGE_ROTATIONS = 20.0;
        public static final double BAR_1_EXTEND_ROTATIONS = 50.0;
        public static final double BAR_1_ENGAGE_ROTATIONS = -20.5;
        public static final double BAR_2_EXTEND_ROTATIONS = 35.0;
        public static final double BAR_2_ENGAGE_ROTATIONS = -20.5;
        public static final double BAR_3_EXTEND_ROTATIONS = 33.0;
        public static final double BAR_3_ENGAGE_ROTATIONS = -20.5;
        public static final double POSITION_TOLERANCE_ROTATIONS = 1.0;
        public static final double MAX_TELESCOPE_LENGTH = 0.718;
        public static final double MIN_TELESCOPE_LENGTH = 0.4771;
        public static final double SIDE_HOOK_HORIZONTAL_LENGTH = 0.100;
        public static final double HOOK_MOUNT_HEIGHT_METERS = 0.2667;
        public static final double HOOK_OFFSET_METERS = 0.1000;
        public static final double TELESCOPE_SIDE_OFFSET_METERS = 0.340;
        public static final int THROUGH_BORE_ENCODER_DIO_CHANNEL = DIOChannel.FOUR.getChannel();
        public static final double THROUGH_BORE_STORED_ANGLE_ROTATIONS = 0.0;
        public static final double THROUGH_BORE_ANGLE_TOLERANCE_ROTATIONS = 0.02;
    }

    public class TurnEncoderOffsets {
        public static final double flTurnEncoderOffset = 4.911;
        public static final double frTurnEncoderOffset = 1.697;
        public static final double blTurnEncoderOffset = -0.100;
        public static final double brTurnEncoderOffset = 3.790;
    }
}
