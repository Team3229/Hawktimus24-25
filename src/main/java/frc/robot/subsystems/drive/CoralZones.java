package frc.robot.subsystems.drive;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.constants.ReefPositions;

public class CoralZones {

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

	public Pose2d findCoralZone(
        boolean leftSide,
        Pose2d robotPose
    ) {
		
		reefCenterPose = FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.Center.getPosition());

		System.out.println("Finding Coral Zone");

		//LEFT SIDE
		if (leftSide && inZone_AL_BC(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone AL_BC");
			return getReefPose("A");
		}
		if (leftSide && inZone_BC_DE(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone BC_DE");
			return getReefPose("C");
		}
		if (leftSide && inZone_DE_FG(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone DE_FG");
			return getReefPose("F");
		}
		if (leftSide && inZone_FG_HI(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone FG_HI");
			return getReefPose("H");
		}
		if (leftSide && inZone_HI_JK(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone HI_JK");
			return getReefPose("J");
		}
		if (leftSide && inZone_JK_AL(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone JK_AL");
			return getReefPose("K");
		}
		

		//RIGHT SIDE
		if (!leftSide && inZone_AL_BC(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone AL_BC");
			return getReefPose("B");
		}
		if (!leftSide && inZone_BC_DE(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone BC_DE");
			return getReefPose("D");
		}
		if (!leftSide && inZone_DE_FG(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone DE_FG");
			return getReefPose("E");
		}
		if (!leftSide && inZone_FG_HI(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone FG_HI");
			return getReefPose("G");
		}
		if (!leftSide && inZone_HI_JK(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone HI_JK");
			return getReefPose("I");
		}
		if (!leftSide && inZone_JK_AL(robotPose, Alliance.getAlliance() == AllianceColor.Blue)) {
			System.out.println("Robot is in zone JK_AL");
			return getReefPose("L");
		}

		return null;
	}		

    private Pose2d getReefPose(String reef) {
        return ReefPositions.valueOf(reef).getPosition();
    }


	private boolean inZone_AL_BC(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getY() > bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getY() < bc(robotPose.getX()));
	}

	private boolean inZone_BC_DE(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < bc(robotPose.getX())) && 
				(robotPose.getX() < de());
		}
		return
			(robotPose.getY() > bc(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

	private boolean inZone_DE_FG(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private boolean inZone_FG_HI(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > al(robotPose.getX())) && 
				(robotPose.getY() < bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getY() > bc(robotPose.getX()));
	}

	private boolean inZone_HI_JK(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > bc(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private boolean inZone_JK_AL(Pose2d robotPose, boolean blue) {
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
