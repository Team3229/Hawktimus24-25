package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
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
            boolean rightSide,
            DriveSubsystem driveSubsystem
        ) {

		Command out = Commands.none();

        System.out.println("Finding Coral Zone");

		//LEFT SIDE
		if (rightSide && inZone_AL_BC(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone AL_BC");
			out = driveToReef("A");
		}
		if (rightSide && inZone_BC_DE(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone BC_DE");
			out = driveToReef("C");
		}
		if (rightSide && inZone_DE_FG(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone DE_FG");
			out = driveToReef("F");
		}
		if (rightSide && inZone_FG_HI(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone FG_HI");
			out = driveToReef("H");
		}
		if (rightSide && inZone_HI_JK(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone HI_JK");
			out = driveToReef("J");
		}
		if (rightSide && inZone_JK_AL(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone JK_AL");
			out = driveToReef("K");
		}
		

		//RIGHT SIDE
		if (!rightSide && inZone_AL_BC(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone AL_BC");
			out = driveToReef("B");
		}
		if (!rightSide && inZone_BC_DE(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone BC_DE");
			out = driveToReef("D");
		}
		if (!rightSide && inZone_DE_FG(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone DE_FG");
			out = driveToReef("E");
		}
		if (!rightSide && inZone_FG_HI(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone FG_HI");
			out = driveToReef("G");
		}
		if (!rightSide && inZone_HI_JK(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone HI_JK");
			out = driveToReef("I");
		}
		if (!rightSide && inZone_JK_AL(driveSubsystem.getPose())) {
            System.out.println("Robot is in zone JK_AL");
			out = driveToReef("L");
		}

		out.addRequirements(driveSubsystem);

		return out;

	}

    private static Command driveToReef(String reef) {
        try {
            return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(reef),
                PathFindingConstraints.SEMIAUTO_ROBOT_CONSTRAINTS
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
