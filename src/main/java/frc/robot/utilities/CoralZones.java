package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class CoralZones {
    
    private static double al(double x) {
		return -(1 / Math.sqrt(3)) * (x - 3.673422) + 4.496925;
	}

	private static double bc(double x) {
		return (1 / Math.sqrt(3)) * (x - 3.673480) + 3.554821;
	}

	private static double de() {
		return 4.489395;
	}

	public static Command findCoralZone(
            boolean leftSide,
            DriveSubsystem driveSubsystem
        ) {

        System.out.println("Finding Coral Zone");

		//LEFT SIDE
		if (leftSide && inZone_AL_BC(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone AL_BC");
			return driveToReef("A");
		}
		if (leftSide && inZone_BC_DE(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone BC_DE");
			return driveToReef("C");
		}
		if (leftSide && inZone_DE_FG(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone DE_FG");
			return driveToReef("F");
		}
		if (leftSide && inZone_FG_HI(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone FG_HI");
			return driveToReef("H");
		}
		if (leftSide && inZone_HI_JK(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone HI_JK");
			return driveToReef("J");
		}
		if (leftSide && inZone_JK_AL(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone JK_AL");
			return driveToReef("K");
		}
		

		//RIGHT SIDE
		if (!leftSide && inZone_AL_BC(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone AL_BC");
			return driveToReef("B");
		}
		if (!leftSide && inZone_BC_DE(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone BC_DE");
			return driveToReef("D");
		}
		if (!leftSide && inZone_DE_FG(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone DE_FG");
			return driveToReef("E");
		}
		if (!leftSide && inZone_FG_HI(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone FG_HI");
			return driveToReef("G");
		}
		if (!leftSide && inZone_HI_JK(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone HI_JK");
			return driveToReef("I");
		}
		if (!leftSide && inZone_JK_AL(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone JK_AL");
			return driveToReef("L");
		}

		System.out.println("Robot is not positioned inside a zone");
		return Commands.none();

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

	

	private static boolean inZone_AL_BC(Pose2d robotPose) {
		return 
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getY() > bc(robotPose.getX()));
	}

	private static boolean inZone_BC_DE(Pose2d robotPose) {
		return 
			(robotPose.getY() < bc(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

	private static boolean inZone_DE_FG(Pose2d robotPose) {
		return
			(robotPose.getY() < al(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

	private static boolean inZone_FG_HI(Pose2d robotPose) {
		return
            (robotPose.getY() > al(robotPose.getX())) && 
            (robotPose.getY() < bc(robotPose.getX()));
			
	}

	private static boolean inZone_HI_JK(Pose2d robotPose) {
		return 
			(robotPose.getY() > bc(robotPose.getX())) && 
			(robotPose.getX() > de());
	}

	private static boolean inZone_JK_AL(Pose2d robotPose) {
		return
			(robotPose.getY() > al(robotPose.getX())) && 
			(robotPose.getX() < de());
	}

}
