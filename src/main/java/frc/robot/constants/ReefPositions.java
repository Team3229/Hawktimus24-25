package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.100, 3.907, Rotation2d.fromDegrees(270))),
    B(new Pose2d(3.100, 3.573, Rotation2d.fromDegrees(270))),
    C(new Pose2d(3.902, 2.745, Rotation2d.fromDegrees(330))),
    D(new Pose2d(4.231, 2.689, Rotation2d.fromDegrees(330))),
    E(new Pose2d(5.242, 2.966, Rotation2d.fromDegrees(30))),
    F(new Pose2d(5.530, 3.120, Rotation2d.fromDegrees(30))),
    G(new Pose2d(5.790, 4.152, Rotation2d.fromDegrees(90))),
    H(new Pose2d(5.790, 4.492, Rotation2d.fromDegrees(90))),
    I(new Pose2d(4.985, 5.235, Rotation2d.fromDegrees(150))),
    J(new Pose2d(4.740, 5.376, Rotation2d.fromDegrees(150))),
    K(new Pose2d(3.684, 5.059, Rotation2d.fromDegrees(210))),
    L(new Pose2d(3.466, 4.913, Rotation2d.fromDegrees(210))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0))),
    HPST(new Pose2d(1.079, 7.040, Rotation2d.fromDegrees(-54))),
    HPSB(new Pose2d(1.079, 0.987, Rotation2d.fromDegrees(54)));

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
