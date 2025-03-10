package frc.robot.utilities;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.hawklibraries.utilities.Alliance;
import frc.robot.subsystems.DriveSubsystem;

public class CoralStationPathing {

    private static double at16To21(double x) {
		return 4.025900;
	}

    public static Command findHumanZones(DriveSubsystem driveSubsystem) {

        Command out = Commands.none();

        if (Alliance.getAlliance() == AllianceColor.Blue) {

            if (aboveZone(driveSubsystem.getPose())) {
                System.out.println("Robot is above zone");
                out = driveToPlayerStation("HPS Top"); 
            }

            System.out.println("Robot is bellow zone");
            out = driveToPlayerStation("HPS Bottom");
        } 

        if (aboveZone(driveSubsystem.getPose())) {
            System.out.println("Robot is above zone");
            out = driveToPlayerStation("HPS Bottom"); //these are flipped as red
        }

        System.out.println("Robot is bellow zone");
        out = driveToPlayerStation("HPS Top"); //these are flipped as red

        out.addRequirements(driveSubsystem);

        return out;

    }

    private static Command driveToPlayerStation(String hps) {
        try {
            return AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile(hps),
                PathFindingConstraints.SEMIAUTO_ROBOT_CONSTRAINTS
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
