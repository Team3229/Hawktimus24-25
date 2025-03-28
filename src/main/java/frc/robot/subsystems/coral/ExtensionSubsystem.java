package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Handles raising the extension for the intake.
 */
public class ExtensionSubsystem extends SubsystemBase {

    private static final int RIGHT_ID = 2;
    private static final int LEFT_ID = 0;
    private static final double LEFT_SERVO_ANGLE = 85;
    private static final double RIGHT_SERVO_ANGLE = 95;

    private Servo left;
    private Servo right;
    
    /**
     * Instantiate our servos and set their angle on init.
     */
    public ExtensionSubsystem() {

        super();

        left = new Servo(LEFT_ID);
        right = new Servo(RIGHT_ID);

        setDefaultCommand(setAngles());

    }

    private Command setAngles() {
        return run(
            () -> {
                left.setAngle(LEFT_SERVO_ANGLE);
                right.setAngle(RIGHT_SERVO_ANGLE);
            }  
        )
        .withName("Extension/Lifting extension");
    }
}
