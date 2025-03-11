package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.190, 4.180, Rotation2d.fromDegrees(180))),
    B(new Pose2d(3.190, 3.860, Rotation2d.fromDegrees(180))),
    C(new Pose2d(3.685, 2.981, Rotation2d.fromDegrees(240))),
    D(new Pose2d(3.970, 2.810, Rotation2d.fromDegrees(240))),
    E(new Pose2d(4.990, 2.800, Rotation2d.fromDegrees(300))),
    F(new Pose2d(5.280, 2.970, Rotation2d.fromDegrees(300))),
    G(new Pose2d(5.800, 3.850, Rotation2d.fromDegrees(000))),
    H(new Pose2d(5.800, 4.180, Rotation2d.fromDegrees(000))),
    I(new Pose2d(5.280, 5.080, Rotation2d.fromDegrees(060))),
    J(new Pose2d(5.000, 5.240, Rotation2d.fromDegrees(060))),
    K(new Pose2d(3.990, 5.250, Rotation2d.fromDegrees(120))),
    L(new Pose2d(3.690, 5.080, Rotation2d.fromDegrees(120))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0))),
    HPST(new Pose2d(1.038, 6.949, Rotation2d.fromDegrees(-54.011))),
    HPSB(new Pose2d(0.890, 1.240, Rotation2d.fromDegrees(54.011)));

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
