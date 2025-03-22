package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.coral.ElevatorSubsystem;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefHeight {
    Base(ElevatorSubsystem.ELEVATOR_BASE_HEIGHT),
    L1(Feet.of(1).plus(Inch.of(6))),
    L2(Feet.of( 2).plus(Inch.of(11 + (5/8)))),
    L3(Inch.of(50.7)),
    L4(Feet.of(6).plus(Inch.of(2)));

    Distance height;

    /**
     * Method to retrieve the height in inches
     * @return the height in inches
     */
    public double getHeightInInches() {
        return height.in(Inch);
    }

}