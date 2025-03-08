package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class VisionSubsystem {

    public static PoseEstimate getMT2Pose(Rotation2d robotRotation, double robotRotationRate) {

        LimelightHelpers.SetRobotOrientation("limelight", robotRotation.getDegrees(), 0, 0, 0, 0, 0);

        PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 != null && Math.abs(robotRotationRate) < 720 && mt2.tagCount != 0) {
            return mt2;
        } else {
            return null;
        }
        
    }
}