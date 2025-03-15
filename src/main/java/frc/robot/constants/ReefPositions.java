package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.144, 3.962, Rotation2d.fromDegrees(270))),
    B(new Pose2d(3.144, 3.628, Rotation2d.fromDegrees(270))),
    C(new Pose2d(3.873, 2.834, Rotation2d.fromDegrees(330))),
    D(new Pose2d(4.161, 2.666, Rotation2d.fromDegrees(330))),
    E(new Pose2d(5.215, 2.894, Rotation2d.fromDegrees(30))),
    F(new Pose2d(5.504, 3.061, Rotation2d.fromDegrees(30))),
    G(new Pose2d(5.830, 4.088, Rotation2d.fromDegrees(90))),
    H(new Pose2d(5.830, 4.420, Rotation2d.fromDegrees(90))),
    I(new Pose2d(5.105, 5.219, Rotation2d.fromDegrees(150))),
    J(new Pose2d(4.818, 5.385, Rotation2d.fromDegrees(150))),
    K(new Pose2d(3.760, 5.153, Rotation2d.fromDegrees(210))),
    L(new Pose2d(3.474, 4.987, Rotation2d.fromDegrees(210))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0))),
    HPST(new Pose2d(1.079, 7.040, Rotation2d.fromDegrees(-54))),
    HPSB(new Pose2d(1.079, 0.987, Rotation2d.fromDegrees(54))),

    ReefNorthWest(new Pose2d(3.850, 5.16, Rotation2d.fromDegrees(30))), //K+L
    ReefWest(new Pose2d(3.174, 4.043, Rotation2d.fromDegrees(90))), //A+B
    ReefSouthWest(new Pose2d(3.8, 2.91, Rotation2d.fromDegrees(150))), //C+D
    ReefSouthEast(new Pose2d(5.16, 2.9, Rotation2d.fromDegrees(210))), //E+F
    ReefEast(new Pose2d(5.8, 3.98, Rotation2d.fromDegrees(270))), //G+H
    ReefNorthEast(new Pose2d(5.190, 5.130, Rotation2d.fromDegrees(330))); //I+J

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
