package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.List;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;

public class CageLineup {

    private static final Pose2d leftCage = new Pose2d(8.773871, 7.259888, new Rotation2d());
    private static final Pose2d midCage = new Pose2d(8.773871, 6.169278, new Rotation2d());
    private static final Pose2d rightCage = new Pose2d(8.773871, 5.078665, new Rotation2d());

    private static final Transform2d cageVertToRobotCenter = new Transform2d(-Inches.of(11+(27/2)).in(Meters), 0, new Rotation2d());

    DriveSubsystem driveSubsystem;

    public static Command cageLineupToNearest(DriveSubsystem driveSubsystem) {
        return driveSubsystem.driveToPose(
            () -> {
                return driveSubsystem.getPose().nearest(
                    List.of(
                        FieldMirroringUtils.toCurrentAlliancePose(leftCage),
                        FieldMirroringUtils.toCurrentAlliancePose(midCage),
                        FieldMirroringUtils.toCurrentAlliancePose(rightCage)
                    )
                ).transformBy(cageVertToRobotCenter);
            }
        );
    }

}
