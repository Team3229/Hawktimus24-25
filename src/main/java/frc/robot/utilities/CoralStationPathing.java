package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.DriveSubsystem;

public class CoralStationPathing {

    private static double at16To21(double x) {
		return 4.025900;
	}

    public static Command findHumanZones(DriveSubsystem driveSubsystem) {

        if (aboveZone(driveSubsystem.getPose())) {
            System.out.println("Robot is above zone");
			return driveToPlayerStation("HPS Top"); 
		}

        System.out.println("Robot is below zone");
        return driveToPlayerStation("HPS Bottom");

    }

    private static Command driveToPlayerStation(String hps) {
        try {

            PathConstraints constraints = new PathConstraints(
			    4, 4.0,
			    4, Units.degreesToRadians(720)
		    );

            return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(hps),
                constraints
            );

        } catch (Exception e) {
            e.printStackTrace();
            return Commands.none();
        }
    }

    private static boolean aboveZone(Pose2d robotPose) {
		return (robotPose.getY() > at16To21(robotPose.getX()));
	}
    
}
