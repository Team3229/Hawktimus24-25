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
        return readyForRemoval(true);
    }

    /**
     * Command to remove algae from the upper part of the reef, then throws it off
     * the back of the robot
     * 
     * @return Command to remove algae from the upper part of the reef
     */
    public Command removeUpperAlgae() {
        return readyForRemoval(false);
    }
    public Command throwUpperAlgae(){
        return arm.upperAlgaeRemovalPosition();
    }

    /**
     * Command to get ready to remove algae from the reef
     * 
     * @param clockwise Whether the wheel should spin clockwise or counterclockwise
     * @return Command to remove algae from the reef
     */
    public Command readyForRemoval(boolean clockwise) {
        return arm.upToRemoveAlgaeFromReef()
        .andThen(wheel.spin(clockwise));
    }

    /**
     * Command to get ready to collect algae from the ground
     * 
     * @return Command to collect algae
     */
    public Command readyForCollection() {
        return arm.readyingCollectPosition()
                .alongWith(wheel.spin(true));
    }

    /**
     * Command to return the arm and wheel to their home positions.
     * 
     * @return
     */
    public Command home() {
        return Commands.parallel(arm.home(), wheel.stop());
    }

    /**
     * Command to hold algae in the arm
     * 
     * @return Command to hold algae
     */
    public Command intakeAlgae() {
        return arm.intakePosition()
                .andThen(new WaitCommand(1))
                .andThen(wheel.stop());
    }

    /**
     * Command to score algae
     * 
     * @return Command to score algae
     */
    public Command scoreAlgae() {
        return Commands.parallel(arm.scorePosition(), wheel.spin(false))
                 .handleInterrupt(() -> wheel.stopWheel());
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
        builder.addDoubleProperty("Arm Angle", () -> arm.getRawPosition().in(Degrees), null);
        builder.addDoubleProperty("Arm Current", () -> arm.getDraw(), null);
        builder.addDoubleProperty("Wheel Current", () -> wheel.getDraw(), null);
        builder.addDoubleProperty("Relative Angle", () -> arm.getRelativePosition().in(Degrees), null);
    }
}
