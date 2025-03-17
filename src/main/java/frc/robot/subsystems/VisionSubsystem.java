package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;

public class VisionSubsystem {

    public static PoseEstimate getMT2Pose(Rotation2d robotRotation, double robotRotationRate) {

        LimelightHelpers.SetRobotOrientation("limelight", robotRotation.getDegrees(), robotRotationRate, 0, 0, 0, 0);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 != null && Math.abs(robotRotationRate) < 720 && mt2.tagCount != 0) {
            return mt2;
        } else {
            return null;
        }
        
    }

    public static Rotation2d getMT1Rotation() {

        PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

        if (mt1 != null && mt1.rawFiducials.length > 0 && mt1.rawFiducials[0].ambiguity < 0.5) {
            return mt1.pose.getRotation();
        } else {
            return null;
        }

    }
}