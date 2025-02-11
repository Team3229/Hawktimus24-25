package frc.robot.subsystems.coral;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.ReefHeight;

/**
 * Connection between: the Spitter, Elevator, and Catcher Subsystems for the
 * manipuler functions.
 */
public class CoralSubsystem extends SubsystemBase {

    ElevatorSubsystem elevatorSubsystem;
    SpitterSubsystem spitterSubsystem;
    CatcherSubsystem catcherSubsystem;
    
    public CoralSubsystem(Trigger catchTrigger) {
        elevatorSubsystem = new ElevatorSubsystem();
        catcherSubsystem = new CatcherSubsystem();
        spitterSubsystem = new SpitterSubsystem();

        // catcherSubsystem.getCoral().onTrue(catcherSubsystem.feedAngle()
        //         .alongWith(spitterSubsystem.intake())
        //         .andThen(catcherSubsystem.catchAngle()));

        catchTrigger.onTrue(catcherSubsystem.feedAngle()
                .alongWith(spitterSubsystem.intake())
                .andThen(catcherSubsystem.catchAngle()));
    }

    /**
     * Connect elevator to spit
     */
    public Command elevatorSpit(ReefHeight reefHeight) {
        return elevatorSubsystem.goToLevel(reefHeight)
            .andThen(spitterSubsystem.spit())
            .andThen(elevatorSubsystem.goToLevel(ReefHeight.Base)
        );
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Spitter has Coral", spitterSubsystem::hasCoral, null);
        builder.addBooleanProperty("Catcher has Coral", catcherSubsystem::hasCoral, null);
    }
}