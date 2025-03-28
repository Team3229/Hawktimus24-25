package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

/**
 * Subsystem for the algae removal and intake.
 */
public class AlgaeSubsystem extends SubsystemBase {
    
    private AlgaeArmSubsystem arm;
    private AlgaeWheelSubsystem wheel;

    public AlgaeSubsystem() {
        arm = new AlgaeArmSubsystem();
        wheel = new AlgaeWheelSubsystem();
    }

    /**
     * Command to remove algae from the lower part of the reef, then drop it
     * 
     * @return
     */
    public Command removeLowerAlgae() {
        return readyForRemoval(true)
            .withName("Algae/Remove lower algae");
    }

    /**
     * Command to remove algae from the upper part of the reef, then throws it off
     * the back of the robot
     * 
     * @return Command to remove algae from the upper part of the reef
     */
    public Command removeUpperAlgae() {
        return readyForRemoval(false)
            .withName("Algae/Remove upper algae");
    }
    public Command throwUpperAlgae(){
        return arm.rotateTo(AlgaeArmSubsystem.THROW_ANGLE)
            .withName("Algae/Throw upper algae");
    }

    /**
     * Command to get ready to remove algae from the reef
     * 
     * @param clockwise Whether the wheel should spin clockwise or counterclockwise
     * @return Command to remove algae from the reef
     */
    public Command readyForRemoval(boolean clockwise) {
        return arm.rotateTo(AlgaeArmSubsystem.REMOVAL_POSITION)
        .andThen(wheel.spin(clockwise))
            .withName("Algae/Ready to remove");
    }

    /**
     * Command to get ready to collect algae from the ground
     * 
     * @return Command to collect algae
     */
    public Command readyForCollection() {
        return arm.rotateTo(AlgaeArmSubsystem.GROUND_COLLECT_ANGLE)
                .alongWith(wheel.spin(true))
                .withName("Algae/Ready to collect");
    }

    /**
     * Command to return the arm and wheel to their home positions.
     * 
     * @return
     */
    public Command home() {
        return Commands.parallel(arm.rotateTo(AlgaeArmSubsystem.HOME_POSITION), wheel.stop())
            .withName("Algae/Home arm");
    }

    /**
     * Command to hold algae in the arm
     * 
     * @return Command to hold algae
     */
    public Command intakeAlgae() {  
        return arm.rotateTo(AlgaeArmSubsystem.HOLD_ANGLE)
                .andThen(new WaitCommand(1))
                .andThen(wheel.stop())
                .withName("Algae/Intake algae");
    }

    /**
     * Command to score algae
     * 
     * @return Command to score algae
     */
    public Command scoreAlgae() {
        return Commands.parallel(arm.rotateTo(AlgaeArmSubsystem.SCORE_ANGLE), wheel.spin(false))
                 .handleInterrupt(() -> wheel.stopWheel())
                 .withName("Algae/Score algae");
    }

    public Command disableAlgaeArm() {
        return 
            wheel.stop()
            .ignoringDisable(true);
    }

    /**
     * Get the position of the arm.
     * 
     * @return double representing the position of the arm
     */
    public Angle getPosition() {
        return arm.getPosition();
    }


    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Arm Angle", () -> arm.getPosition().in(Degrees), null);
        builder.addDoubleProperty("Wheel Current", () -> wheel.getDraw(), null);
    }
}
