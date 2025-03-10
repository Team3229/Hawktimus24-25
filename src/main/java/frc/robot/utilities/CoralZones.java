package frc.robot.utilities;

import org.ironmaple.utils.FieldMirroringUtils;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.constants.ReefPositions;
import frc.robot.subsystems.DriveSubsystem;

public class CoralZones {

	private static Pose2d reefCenterPose = ReefPositions.Center.getPosition();
    
    private static double al(double x) {
		return -(1 / Math.sqrt(3)) * (x - reefCenterPose.getX()) + reefCenterPose.getY();
	}

	private static double bc(double x) {
		return (1 / Math.sqrt(3)) * (x - reefCenterPose.getX()) + reefCenterPose.getY();
	}

	private static double de() {
		return reefCenterPose.getX();
	}

	public static Command findCoralZone(
        boolean leftSide,
        DriveSubsystem driveSubsystem
    ) {
		return Commands.runOnce(() -> {
			reefCenterPose = FieldMirroringUtils.toCurrentAlliancePose(ReefPositions.Center.getPosition());

			System.out.println("Finding Coral Zone");

			//LEFT SIDE
			if (leftSide && inZone_AL_BC(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone AL_BC");
				driveToReef("A").schedule();
			}
			if (leftSide && inZone_BC_DE(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone BC_DE");
				driveToReef("C").schedule();
			}
			if (leftSide && inZone_DE_FG(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone DE_FG");
				driveToReef("F").schedule();
			}
			if (leftSide && inZone_FG_HI(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone FG_HI");
				driveToReef("H").schedule();
			}
			if (leftSide && inZone_HI_JK(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone HI_JK");
				driveToReef("J").schedule();
			}
			if (leftSide && inZone_JK_AL(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone JK_AL");
				driveToReef("K").schedule();
			}
			

			//RIGHT SIDE
			if (!leftSide && inZone_AL_BC(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone AL_BC");
				driveToReef("B").schedule();
			}
			if (!leftSide && inZone_BC_DE(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone BC_DE");
				driveToReef("D").schedule();
			}
			if (!leftSide && inZone_DE_FG(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone DE_FG");
				driveToReef("E").schedule();
			}
			if (!leftSide && inZone_FG_HI(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone FG_HI");
				driveToReef("G").schedule();
			}
			if (!leftSide && inZone_HI_JK(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone HI_JK");
				driveToReef("I").schedule();
			}
			if (!leftSide && inZone_JK_AL(driveSubsystem.getPose(), Alliance.getAlliance() == AllianceColor.Blue)) {
				System.out.println("Robot is in zone JK_AL");
				driveToReef("L").schedule();
			}
		}
	);
}
		

    private static Command driveToReef(String reef) {
        try {

            PathConstraints constraints = new PathConstraints(
			    4, 4.0,
			    4, Units.degreesToRadians(720)
		    );

            return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(reef),
                constraints
            );

        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

	

	private static boolean inZone_AL_BC(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getY() > bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getY() < bc(robotPose.getX()));
	}

	private static boolean inZone_BC_DE(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < bc(robotPose.getX())) && 
				(robotPose.getX() < de());
		}
		return
			(robotPose.getY() > bc(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

	private static boolean inZone_DE_FG(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() < al(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private static boolean inZone_FG_HI(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > al(robotPose.getX())) && 
				(robotPose.getY() < bc(robotPose.getX()));
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getY() > bc(robotPose.getX()));
	}

	private static boolean inZone_HI_JK(Pose2d robotPose, boolean blue) {
		if(blue){
			return 
				(robotPose.getY() > bc(robotPose.getX())) && 
				(robotPose.getX() > de());
		} 
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private static boolean inZone_JK_AL(Pose2d robotPose, boolean blue) {
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
