package frc.robot.subsystems.coral;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

        super();

        elevatorSubsystem = new ElevatorSubsystem();
        catcherSubsystem = new CatcherSubsystem();
        spitterSubsystem = new SpitterSubsystem();

        // catcherSubsystem.getCoral().onTrue(catcherSubsystem.feedAngle()
        //         .alongWith(spitterSubsystem.intake())
        //         .andThen(catcherSubsystem.catchAngle()));

        catchTrigger.onTrue(catcherSubsystem.feedAngle()
                .alongWith(spitterSubsystem.intake())
                .andThen(catcherSubsystem.catchAngle()));

        NamedCommands.registerCommand("L4", elevatorSpit(ReefHeight.L4));
        NamedCommands.registerCommand("L3", elevatorSpit(ReefHeight.L3));
        NamedCommands.registerCommand("L2", elevatorSpit(ReefHeight.L2));
        NamedCommands.registerCommand("L1", elevatorSpit(ReefHeight.L1));
        NamedCommands.registerCommand("Wait for Intake", Commands.waitUntil(catcherSubsystem::hasCoral).withTimeout(2));
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

    public Command spit () {
        return spitterSubsystem.spit();
    }

    public Distance getElevatorPos() {
        return elevatorSubsystem.getElevatorPos();
    }

    public double getFeederAngle() {
        return catcherSubsystem.getFeederAngle();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Spitter has Coral", spitterSubsystem::hasCoral, null);
        builder.addBooleanProperty("Catcher has Coral", catcherSubsystem::hasCoral, null);
    }
}