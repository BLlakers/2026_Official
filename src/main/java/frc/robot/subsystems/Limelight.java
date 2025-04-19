package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.support.limelight.LimelightHelpers;

public class Limelight extends SubsystemBase {
    private final DoubleArraySubscriber aprilTagPoseTopic;
    private final IntegerPublisher priorityTagIdPub;
    private final String limelightName;
    private AprilTag currentAprilTag = new AprilTag(-1, new Pose3d());
    private double getPriorityID;

    public Limelight() {
        this("limelight");
    }

    public Limelight(String cameraName) {
        limelightName = cameraName;
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        aprilTagPoseTopic = table.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[]{0, 0, 0, 0, 0, 0});
        priorityTagIdPub = table.getIntegerTopic("priorityid").publish();
        getPriorityID = table.getEntry("priorityID").getDouble(-1); // Default to -1 if no value is found
    }

    public void SetTagIDToTrack(int tagID) {
        priorityTagIdPub.accept(tagID);

    }

    @Override
    public void periodic() {
        currentAprilTag = getCurrentAprilTag();
        SmartDashboard.putNumber("AprilTag/tagID", currentAprilTag.ID);
        SmartDashboard.putNumber("AprilTag/pose/X", currentAprilTag.pose.getX());
        SmartDashboard.putNumber("AprilTag/pose/Y", currentAprilTag.pose.getY());
        SmartDashboard.putNumber("AprilTag/pose/Z", currentAprilTag.pose.getZ());
        SmartDashboard.putNumber("AprilTag/pose/rotX", Math.toDegrees(currentAprilTag.pose.getRotation().getX()));
        SmartDashboard.putNumber("AprilTag/pose/rotY", Math.toDegrees(currentAprilTag.pose.getRotation().getY()));
        SmartDashboard.putNumber("AprilTag/pose/rotZ", Math.toDegrees(currentAprilTag.pose.getRotation().getZ()));
        SmartDashboard.putNumber("AprilTag/pose/measureRotZ", currentAprilTag.pose.getRotation().getMeasureZ().magnitude());
        SmartDashboard.putNumber("AprilTag/pose/GetAngle", currentAprilTag.pose.getRotation().getAngle());
        SmartDashboard.putNumber("AprilTag/pose/GetAxis", currentAprilTag.pose.getRotation().getAxis().get(2));
        //SmartDashboard.putNumber("AprilTag/pose/RAWz", aprilTagPoseTopic.getAtomic().value[5]);
        SmartDashboard.putNumber("AprilTag/pose/RAWZFromMethod", getAprilRotation2d().getDegrees());

    }

    /**
     * This function gets the april tag the camera is viewing.
     *
     * @return The AprilTag the camera is looking at plus a Pose3d - x, y, z, ROTx, ROTy, ROTz.
     */
    public AprilTag getCurrentAprilTag() {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);

        NetworkTableEntry tid = table.getEntry("tid");
        int aprilTagId = (int) tid.getInteger(-1);
        TimestampedDoubleArray poseArray = aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)

        if (poseArray.value.length < 6) return new AprilTag(-1, new Pose3d());

        Translation3d poseTranslation = new Translation3d(poseArray.value[0], // x
                poseArray.value[1], // y
                poseArray.value[2] // z
        );

        Rotation3d poseOrientation = new Rotation3d(poseArray.value[3], // roll = rotx
                poseArray.value[4], // pitch = roty
                poseArray.value[5] // yaw = rotz
        );

        Pose3d aprilTagPose = new Pose3d(poseTranslation, poseOrientation); // creating pose3d based off of our
        // translation3d and rot3d and tid
        return new AprilTag(aprilTagId, aprilTagPose);
    }

    /**
     * This function gets the raw Rotation3d values
     *
     * @return The Rotation2d of aprilTag
     */
    public Rotation2d getAprilRotation2d() {
        TimestampedDoubleArray poseArray = aprilTagPoseTopic.getAtomic(); // (x, y, z, rotx, roty, rotz)

        return new Rotation2d(Math.toRadians(poseArray.value[5]));
    }

    public Command setLimelightUsageField() {
        return Commands.runOnce(() -> LimelightHelpers.setCameraPose_RobotSpace("limelight-frl", 0, 0, 0, 0, -90, 0));
    }

    public Command setLimelightUsageRobot() {
        return Commands.runOnce(() -> LimelightHelpers.setCameraPose_RobotSpace("limelight-frl", .17, 0, .2, 0, 0, 0));
    }

    public Command getPriorityIDCommand(int idBlue, int idRed) {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return this.runOnce(() -> this.SetTagIDToTrack(idRed));

        } else if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Blue) {
            return this.runOnce(() -> this.SetTagIDToTrack(idBlue));
        } else {
            return this.runOnce(() -> this.SetTagIDToTrack(-1));
        }
    }

    public Pose2d getGoalPose(boolean isLeft) {
        boolean m_isLeft = isLeft;
        Pose2d goalPose2d;
        switch (getCurrentAprilTag().ID) {
            case 6:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(13.714, 2.868, new Rotation2d(Math.toRadians(120)));
                } else {
                    goalPose2d = new Pose2d(13.930, 3.012, new Rotation2d(Math.toRadians(120)));
                }
                break;
            case 7:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(14.373, 4.019, new Rotation2d(Math.toRadians(180)));
                } else {
                    goalPose2d = new Pose2d(14.373, 4.199, new Rotation2d(Math.toRadians(180)));
                }
                break;
            case 8:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(13.738, 5.158, new Rotation2d(Math.toRadians(Math.toRadians(-120))));
                } else {
                    goalPose2d = new Pose2d(13.570, 5.266, new Rotation2d((Math.toRadians(Math.toRadians(-120)))));
                }
                break;
            case 9:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(14.373, 5.182, new Rotation2d(Math.toRadians(-60)));
                } else {
                    goalPose2d = new Pose2d(12.263, 5.098, new Rotation2d(Math.toRadians(-60)));
                }
                break;
            case 10:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(11.736, 4.019, new Rotation2d(Math.toRadians(0)));
                } else {
                    goalPose2d = new Pose2d(11.736, 3.839, new Rotation2d(Math.toRadians(0)));
                }
                break;
            case 11:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(12.407, 2.904, new Rotation2d(Math.toRadians(60)));
                } else {
                    goalPose2d = new Pose2d(12.563, 2.796, new Rotation2d(Math.toRadians(60)));
                }
                break;
            case 17:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.824, 2.904, new Rotation2d(Math.toRadians(60)));
                } else {
                    goalPose2d = new Pose2d(3.992, 2.820, new Rotation2d(Math.toRadians(60)));
                }
                break;
            case 18:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.165, 4.031, new Rotation2d(Math.toRadians(0)));
                } else {
                    goalPose2d = new Pose2d(3.177, 3.815, new Rotation2d(Math.toRadians(0)));
                }
                break;
            case 19:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(3.824, 5.170, new Rotation2d(Math.toRadians(-60)));
                } else {
                    goalPose2d = new Pose2d(3.668, 5.086, new Rotation2d(Math.toRadians(-60)));
                }
                break;
            case 20:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.155, 5.170, new Rotation2d(Math.toRadians(-60)));
                } else {
                    goalPose2d = new Pose2d(5.023, 5.242, new Rotation2d(Math.toRadians(-60)));
                }
                break;
            case 21:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.814, 4.019, new Rotation2d(Math.toRadians(180)));
                } else {
                    goalPose2d = new Pose2d(5.814, 4.307, new Rotation2d(Math.toRadians(180)));
                }
                break;
            case 22:
                if (m_isLeft) {
                    goalPose2d = new Pose2d(5.119, 2.880, new Rotation2d(Math.toRadians(120)));
                } else {
                    goalPose2d = new Pose2d(5.394, 3.024, new Rotation2d(Math.toRadians(120)));
                }
                break;
            default:
                goalPose2d = new Pose2d(-99999, 0, new Rotation2d());
                break;
        }
        return goalPose2d;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addDoubleProperty("AprilTag/tagID", () -> currentAprilTag.ID, null);
        builder.addDoubleProperty("AprilTag/pose/X", currentAprilTag.pose::getX, null);
        builder.addDoubleProperty("AprilTag/pose/Y", currentAprilTag.pose::getY, null);
        builder.addDoubleProperty("AprilTag/pose/Z", currentAprilTag.pose::getZ, null);
        builder.addDoubleProperty("AprilTag/pose/rotX", currentAprilTag.pose.getRotation()::getX, null);
        builder.addDoubleProperty("AprilTag/pose/rotY", currentAprilTag.pose.getRotation()::getY, null);
        builder.addDoubleProperty("AprilTag/pose/rotZ", currentAprilTag.pose.getRotation()::getZ, null);
        builder.addDoubleProperty("AprilTag/PriorityID", () -> getPriorityID, null);
    }
}
