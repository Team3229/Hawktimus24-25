package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisualizerSubsystem extends SubsystemBase {

    private StructPublisher<Pose3d> elevatorPosePub;
    private StructPublisher<Pose3d> elevatorMidPosePub;
    private StructPublisher<Pose3d> feederPosePub;
    private StructPublisher<Pose3d> climbPosePub;
    private StructPublisher<Pose3d> algaePosePub;

    private Supplier<Double> elevatorPos;
    private Supplier<Double> feederAngle;
    private Supplier<Double> climbPos;
    private Supplier<Double> algaePos;

    public VisualizerSubsystem(
        Supplier<Double> elevatorPos,
        Supplier<Double> feederAngle,
        Supplier<Double> climbPos,
        Supplier<Double> algaePos
    ) {
        super();

        this.elevatorPos = elevatorPos;
        this.feederAngle = feederAngle;
        this.climbPos = climbPos;
        this.algaePos = algaePos;

        elevatorPosePub = NetworkTableInstance.getDefault()
  .getStructTopic("ElevatorPose", Pose3d.struct).publish();
        elevatorMidPosePub = NetworkTableInstance.getDefault()
  .getStructTopic("ElevatorMidPose", Pose3d.struct).publish();
        feederPosePub = NetworkTableInstance.getDefault()
  .getStructTopic("FeederPose", Pose3d.struct).publish();
        climbPosePub = NetworkTableInstance.getDefault()
    .getStructTopic("ClimbPose", Pose3d.struct).publish();
        algaePosePub = NetworkTableInstance.getDefault()
    .getStructTopic("AlgaePose", Pose3d.struct).publish();

    }

    @Override
    public void periodic() {
        elevatorPosePub.set(new Pose3d(
            0, 0, elevatorPos.get(), new Rotation3d()
        ));

        elevatorMidPosePub.set(new Pose3d(
            0, 0, elevatorPos.get() / 2, new Rotation3d()
        ));

        feederPosePub.set(new Pose3d(
            0.207981042, 0.3345180762, 0.806, new Rotation3d(
                0, Degrees.of(feederAngle.get()).in(Radians), 0
            )
        ));

        climbPosePub.set(new Pose3d(
            0.0101351896, 0.0935954986, 0.055889017, new Rotation3d(
                Degrees.of(climbPos.get()).in(Radians), 0, Degrees.of(28).in(Radians)
            )
        ));

        algaePosePub.set(new Pose3d(
            -0.274618958, 0.0902502386, 0.7516080472, new Rotation3d(
                0, Degrees.of(algaePos.get()).in(Radians), 0
            )
        ));
    }

}
