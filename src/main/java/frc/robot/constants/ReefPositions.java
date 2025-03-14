package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.174, 3.962, Rotation2d.fromDegrees(270))),
    B(new Pose2d(3.174, 3.628, Rotation2d.fromDegrees(270))),
    C(new Pose2d(3.888, 2.872, Rotation2d.fromDegrees(330))),
    D(new Pose2d(4.180, 2.715, Rotation2d.fromDegrees(330))),
    E(new Pose2d(5.187, 2.929, Rotation2d.fromDegrees(30))),
    F(new Pose2d(5.480, 3.101, Rotation2d.fromDegrees(30))),
    G(new Pose2d(5.800, 4.088, Rotation2d.fromDegrees(90))),
    H(new Pose2d(5.800, 4.420, Rotation2d.fromDegrees(90))),
    I(new Pose2d(5.083, 5.186, Rotation2d.fromDegrees(150))),
    J(new Pose2d(4.795, 5.346, Rotation2d.fromDegrees(150))),
    K(new Pose2d(3.783, 5.115, Rotation2d.fromDegrees(210))),
    L(new Pose2d(3.493, 4.952, Rotation2d.fromDegrees(210))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0))),
    HPST(new Pose2d(1.079, 7.040, Rotation2d.fromDegrees(-54))),
    HPSB(new Pose2d(1.079, 0.987, Rotation2d.fromDegrees(54))),

    ReefNorthWest(new Pose2d(3.850, 5.16, Rotation2d.fromDegrees(300))), //K+L
    ReefWest(new Pose2d(3.174, 4.6, Rotation2d.fromDegrees(90))), //A+B
    ReefSouthWest(new Pose2d(3.8, 2.91, Rotation2d.fromDegrees(150))), //C+D
    ReefSouthEast(new Pose2d(5.16, 2.9, Rotation2d.fromDegrees(210))), //E+F
    ReefEast(new Pose2d(5.8, 3.98, Rotation2d.fromDegrees(270))), //G+H
    ReefNorthEast(new Pose2d(5.190, 5.130, Rotation2d.fromDegrees(330))); //I+J

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
