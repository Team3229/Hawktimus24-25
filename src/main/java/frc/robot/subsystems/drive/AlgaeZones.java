package frc.robot.subsystems.drive;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
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
        Pose2d robotPose,
		AllianceColor alliance
    ) {
		
		reefCenterPose = FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.Center.getPosition());

		System.out.println("Finding Algae Zone");

		if (inZone_West(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone West");
			return getReefPose("ReefWest", alliance == AllianceColor.Blue);
		}
		if (inZone_SouthWest(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone SouthWest");
			return getReefPose("ReefSouthWest", alliance == AllianceColor.Blue);
		}
		if (inZone_SouthEast(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone SouthEast");
			return getReefPose("ReefSouthEast", alliance == AllianceColor.Blue);
		}
		if (inZone_East(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone East");
			return getReefPose("ReefEast", alliance == AllianceColor.Blue);
		}
		if (inZone_NorthEast(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone NorthEast");
			return getReefPose("ReefNorthEast", alliance == AllianceColor.Blue);
		}
		if (inZone_NorthWest(robotPose, alliance == AllianceColor.Blue)) {
			System.out.println("Robot is in zone NorthWest");
			return getReefPose("ReefNorthWest", alliance == AllianceColor.Blue);
		}

		return null;
	}		

    private Pose2d getReefPose(String reef, boolean blue) {
		if (blue) {
			return ReefPositions.valueOf(reef).getPosition();
		} else {
			return FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.valueOf(reef).getPosition());
		}
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
