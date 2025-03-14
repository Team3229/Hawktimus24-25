package frc.robot.subsystems.drive;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.constants.ReefPositions;

public class AlgaeZones {

	private Pose2d reefCenterPose = ReefPositions.Center.getPosition();
    
    private double al(double x) {
		return -(1 / Math.sqrt(3)) * (x - reefCenterPose.getX()) + reefCenterPose.getY();
	}

	private double bc(double x) {
		return (1 / Math.sqrt(3)) * (x - reefCenterPose.getX()) + reefCenterPose.getY();
	}

	private double de() {
		return reefCenterPose.getX();
	}

	public Pose2d findAlgaeZone(
        Pose2d robotPose
    ) {
		
		reefCenterPose = FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.Center.getPosition());

		System.out.println("Finding Algae Zone");

		//LEFT SIDE
		if (inZone_West(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone East");
			return getReefPose("ReefWest");
		}
		if (inZone_SouthWest(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone BC_DE");
			return getReefPose("ReefSouthWest");
		}
		if (inZone_SouthEast(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone DE_FG");
			return getReefPose("ReefSouthEast");
		}
		if (inZone_East(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone FG_HI");
			return getReefPose("ReefEast");
		}
		if (inZone_NorthEast(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone HI_JK");
			return getReefPose("ReefNorthEast");
		}
		if (inZone_NorthWest(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone JK_AL");
			return getReefPose("ReefNorthWest");
		}

		return null;
	}		

    private Pose2d getReefPose(String reef) {
        return ReefPositions.valueOf(reef).getPosition();
    }


	private boolean inZone_West(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getY() > bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getY() < bc(robotPose.getX()));
	}

	private boolean inZone_SouthWest(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < bc(robotPose.getX())) && 
				(robotPose.getX() < de());
		}
		return
			(robotPose.getY() > bc(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

	private boolean inZone_SouthEast(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private boolean inZone_East(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > al(robotPose.getX())) && 
				(robotPose.getY() < bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getY() > bc(robotPose.getX()));
	}

	private boolean inZone_NorthEast(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > bc(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private boolean inZone_NorthWest(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > al(robotPose.getX())) && 
				(robotPose.getX() < de());
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

}
