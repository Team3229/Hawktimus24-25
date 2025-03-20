package frc.robot.subsystems.coral;

import edu.wpi.first.wpilibj.Servo;

/**
 * Handles raising the extension for the intake.
 */
public class ExtentionSubsystem {

    private static final int RIGHT_ID = 1;
    private static final int LEFT_ID = 0;
    private static final double LEFT_SERVO_ANGLE = 70;
    private static final double RIGHT_SERVO_ANGLE = 110;

    private Servo left;
    private Servo right;
    
    /**
     * Instantiate our servos and set their angle on init.
     */
    public ExtentionSubsystem() {
        left = new Servo(LEFT_ID);
        right = new Servo(RIGHT_ID);

        left.setAngle(LEFT_SERVO_ANGLE);
        right.setAngle(RIGHT_SERVO_ANGLE);
    }
}
