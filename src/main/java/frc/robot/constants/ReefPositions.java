package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {
    
    A(new Pose2d(3.190, 3.860, Rotation2d.fromDegrees(180))),
    B(new Pose2d(3.190, 3.557, Rotation2d.fromDegrees(180))),
    C(new Pose2d(3.981, 2.821, Rotation2d.fromDegrees(240))),
    D(new Pose2d(4.210, 2.680, Rotation2d.fromDegrees(240))),
    E(new Pose2d(5.280, 2.990, Rotation2d.fromDegrees(300))),
    F(new Pose2d(5.458, 3.092, Rotation2d.fromDegrees(300))),
    G(new Pose2d(5.458, 4.152, Rotation2d.fromDegrees(000))),
    H(new Pose2d(5.790, 4.492, Rotation2d.fromDegrees(000))),
    I(new Pose2d(4.985, 5.235, Rotation2d.fromDegrees(060))),
    J(new Pose2d(4.740, 5.376, Rotation2d.fromDegrees(060))),
    K(new Pose2d(3.684, 5.059, Rotation2d.fromDegrees(120))),
    L(new Pose2d(3.466, 4.913, Rotation2d.fromDegrees(120))),
    Center(new Pose2d(4.489337, 4.025923, Rotation2d.fromDegrees(0)));

    Pose2d position;

    public Pose2d getPosition() {
        return position;
    }

}
