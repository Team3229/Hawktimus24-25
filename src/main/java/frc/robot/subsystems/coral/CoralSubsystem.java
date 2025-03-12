package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ReefHeight;

/**
 * Connection between: the Spitter, Elevator, and Catcher Subsystems for the
 * manipulator functions.
 */
public class CoralSubsystem extends SubsystemBase {

    ElevatorSubsystem elevatorSubsystem;
    SpitterSubsystem spitterSubsystem;
    CatcherSubsystem catcherSubsystem;
    
    public CoralSubsystem() {

        super();

        elevatorSubsystem = new ElevatorSubsystem();
        catcherSubsystem = new CatcherSubsystem();
        spitterSubsystem = new SpitterSubsystem();

        registerCommands();

        catcherSubsystem.hasCoral()
        .debounce(0.25)
        .onTrue(
            feedCommand()
        );

    }

    public Command feedCommand() {
        return
        runOnce(
            () -> System.out.println("Feeding...")
        )
        .andThen(
            Commands.parallel(
                catcherSubsystem.feedAngle(),
                spitterSubsystem.intake()
            )
        )
        .andThen(
            catcherSubsystem.catchAngle()
        )
        .handleInterrupt(
            () -> catcherSubsystem.disableCatcher()
        )
        .withTimeout(
            2
        );
    }

    /**
     * Connect elevator to spit
     */
    public Command elevatorSpit(ReefHeight reefHeight) {
        return
            (
                elevatorSubsystem.goToLevel(reefHeight)
                    .andThen(spitterSubsystem.spit())
                    .andThen(Commands.waitTime(ElevatorSubsystem.L4_EXTRA_WAIT_TIME))
                    .andThen(elevatorSubsystem.goToLevel(ReefHeight.Base))
            )
        .onlyIf(spitterSubsystem.hasCoral());
    }

    public Command spit() {
        return spitterSubsystem.spit();
    }

    public Distance getElevatorPose() {
        return elevatorSubsystem.getElevatorPose();
    }

    public Angle getFeederAngle() {
        return catcherSubsystem.getAngle();
    }

    private void registerCommands() {
        NamedCommands.registerCommand("L4", elevatorSpit(ReefHeight.L4));
        NamedCommands.registerCommand("L3", elevatorSpit(ReefHeight.L3));
        NamedCommands.registerCommand("L2", elevatorSpit(ReefHeight.L2));
        NamedCommands.registerCommand("L1", elevatorSpit(ReefHeight.L1));
    
        NamedCommands.registerCommand("Wait for Intake", 
            Commands.waitUntil(catcherSubsystem.hasCoral())
            .withTimeout(RobotBase.isReal() ? Double.POSITIVE_INFINITY : 2)
        );
    }    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Spitter has Coral", spitterSubsystem.hasCoral(), null);
        builder.addBooleanProperty("Catcher has Coral", catcherSubsystem.hasCoral(), null);
        builder.addDoubleProperty("Elevator Height", () -> elevatorSubsystem.getElevatorPose().in(Inch), null);
        builder.addDoubleProperty("Catcher Angle", () -> getFeederAngle().in(Degrees), null);
        builder.addBooleanProperty("Catcher Limit Switch", () -> catcherSubsystem.limit().getAsBoolean(), null);
        builder.addDoubleProperty("Catcher Current", () -> catcherSubsystem.getDraw(), null);
    }
}