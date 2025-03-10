package frc.robot.utilities;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;

public class PathFindingConstraints {
    
    public static final LinearVelocity MAX_ROBOT_VELOCITY = MetersPerSecond.of(1);
    public static final LinearAcceleration MAX_ROBOT_ACCELERATION = MetersPerSecondPerSecond.of(1);

    public static final AngularVelocity MAX_ROBOT_ANGULAR_VELOCITY = DegreesPerSecond.of(360);
    public static final AngularAcceleration MAX_ROBOT_ANGULAR_ACCELERATION = DegreesPerSecondPerSecond.of(360);

    public static final PathConstraints SEMIAUTO_ROBOT_CONSTRAINTS = 
        new PathConstraints(
            MAX_ROBOT_VELOCITY,
            MAX_ROBOT_ACCELERATION,
            MAX_ROBOT_ANGULAR_VELOCITY,
            MAX_ROBOT_ANGULAR_ACCELERATION
        );

}
