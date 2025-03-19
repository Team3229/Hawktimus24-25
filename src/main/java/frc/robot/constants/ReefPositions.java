package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefPositions {

    A(getRobotPose(18, Sides.LEFT)),
    B(getRobotPose(18, Sides.RIGHT)),
    C(getRobotPose(17, Sides.LEFT)),
    D(getRobotPose(17, Sides.RIGHT)),
    E(getRobotPose(22, Sides.LEFT)),
    F(getRobotPose(22, Sides.RIGHT)),
    G(getRobotPose(21, Sides.LEFT)),
    H(getRobotPose(21, Sides.RIGHT)),
    I(getRobotPose(20, Sides.LEFT)),
    J(getRobotPose(20, Sides.RIGHT)),
    K(getRobotPose(19, Sides.LEFT)),
    L(getRobotPose(19, Sides.RIGHT)),

    Center(
        new Pose2d(
            getRobotPose(21, Sides.CENTER).relativeTo(getRobotPose(18, Sides.CENTER)).getX() / 2 + getRobotPose(21, Sides.CENTER).getX(),
            getRobotPose(21, Sides.CENTER).relativeTo(getRobotPose(18, Sides.CENTER)).getY() / 2 + getRobotPose(21, Sides.CENTER).getY(),
            new Rotation2d(0)
        )
    );
        
    private Pose2d position;

    private static enum Sides {
        LEFT, RIGHT, CENTER
    }

    private static Pose2d getRobotPose(int tagID, Sides side) {

        Pose2d tagPose = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark).getTagPose(tagID).get().toPose2d();

        Transform2d tagToLeftReef = new Transform2d(
            Inches.of(0),
            Inches.of(-6.5),
            new Rotation2d(0)
        );
        Transform2d tagToRightReef = new Transform2d(
            Inches.of(0),
            Inches.of(6.5),
            new Rotation2d(0)
        );

        Transform2d reefToBumper  = new Transform2d(
            Inches.of(3),
            Inches.of(0),
            new Rotation2d(0)
        );
        Transform2d spitterToBot = new Transform2d(
            Inches.of(16.5),
            Inches.of(10.409915),
            new Rotation2d(0)
        );
        Transform2d spitterToBotRotation = new Transform2d(
            0,0, Rotation2d.fromDegrees(0)
        );

        switch (side) {
            case LEFT:
                tagPose = tagPose
                    .transformBy(tagToLeftReef);
                break;
            case RIGHT:
                tagPose = tagPose
                    .transformBy(tagToRightReef);
                break;
            case CENTER:
                return tagPose;
        }

        return tagPose
            .transformBy(reefToBumper)
            .transformBy(spitterToBot)
            .transformBy(spitterToBotRotation);

    }

    public Pose2d getPosition() {
        return position;
    }
}
