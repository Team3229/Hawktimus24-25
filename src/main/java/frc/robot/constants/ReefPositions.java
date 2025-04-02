package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import lombok.AllArgsConstructor;
import lombok.Getter;

/**
 * Calculates the position of the robot relative to the reef for each of the Reef Poles A-L and the center of the reef.
 */
@AllArgsConstructor
@Getter
public enum ReefPositions {

    A(18, Sides.LEFT),
    B(18, Sides.RIGHT),
    C(17, Sides.LEFT),
    D(17, Sides.RIGHT),
    E(22, Sides.LEFT),
    F(22, Sides.RIGHT),
    G(21, Sides.LEFT),
    H(21, Sides.RIGHT),
    I(20, Sides.LEFT),
    J(20, Sides.RIGHT),
    K(19, Sides.LEFT),
    L(19, Sides.RIGHT),

    West(18, Sides.CENTER),
    SouthWest(17, Sides.CENTER),
    SouthEast(22, Sides.CENTER),
    East(21, Sides.CENTER),
    NorthEast(20, Sides.CENTER),
    NorthWest(19, Sides.CENTER),

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

    private ReefPositions(int tagID, Sides side) {
        this(getRobotPose(tagID, side));
    }

    private static Pose2d getRobotPose(int tagID, Sides side) {
        Pose2d tagPose = Field.getField().getTagPose(tagID).get().toPose2d();

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
            Inches.of(10.125),
            new Rotation2d(0)
        );
        Transform2d spitterToBotRotation = new Transform2d(
            0,0, Rotation2d.fromDegrees(90)
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
}
