package frc.robot.constants;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.175, 4.375, Rotation2d.fromDegrees(180))),
    B(new Pose2d(3.175, 4.100, Rotation2d.fromDegrees(180))),
    C(new Pose2d(3.575, 3.080, Rotation2d.fromDegrees(240))),
    D(new Pose2d(3.750, 3.000, Rotation2d.fromDegrees(240))),
    E(new Pose2d(4.775, 2.750, Rotation2d.fromDegrees(300))),
    F(new Pose2d(5.000, 2.850, Rotation2d.fromDegrees(300))),
    G(new Pose2d(5.750, 3.600, Rotation2d.fromDegrees(000))),
    H(new Pose2d(5.750, 3.900, Rotation2d.fromDegrees(000))),
    I(new Pose2d(5.500, 4.900, Rotation2d.fromDegrees(060))),
    J(new Pose2d(5.150, 5.125, Rotation2d.fromDegrees(060))),
    K(new Pose2d(4.125, 5.290, Rotation2d.fromDegrees(120))),
    L(new Pose2d(3.920, 5.180, Rotation2d.fromDegrees(120))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0)));

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
