package frc.robot.subsystems.drive;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.constants.ReefPositions;

public class CoralStationPathing {

    private double centerLine(double x) {
		return 4.025900;
	}

    public Pose2d getTargetStation(Pose2d robotPose) {

        boolean blueAlliance = Alliance.getAlliance() == AllianceColor.Blue;

        if (blueAlliance) {
            return getBlueTarget(robotPose);
        } else {
            return getRedTarget(robotPose);
        }
    }

    private Pose2d getBlueTarget(Pose2d robotPose) {
        if (aboveZone(robotPose)) {
            System.out.println("Robot is above zone");
            return driveToPlayerStation("HPST"); 
        } else {
            System.out.println("Robot is bellow zone");
            return driveToPlayerStation("HPSB");
        }
    }

    private Pose2d getRedTarget(Pose2d robotPose) {
        if (aboveZone(robotPose)) {
            System.out.println("Robot is above zone");
            return driveToPlayerStation("HPSB"); 
        } else {
            System.out.println("Robot is bellow zone");
            return driveToPlayerStation("HPST");
        }
    }

    private Pose2d driveToPlayerStation(String hps) {

        return FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.valueOf(hps).getPosition());
    }

    private boolean aboveZone(Pose2d robotPose) {
		return (robotPose.getY() > centerLine(robotPose.getX()));
	}
    
}
