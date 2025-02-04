package frc.robot.constants;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inch;

import edu.wpi.first.units.measure.Distance;
import lombok.AllArgsConstructor;
import lombok.Getter;

@AllArgsConstructor
@Getter
public enum ReefHeight {
    L1(Feet.of(1).plus(Inch.of(6))),
    L2(Feet.of( 2).plus(Inch.of(11 + (5/8)))),
    L3(Feet.of(4).plus(Inch.of(4))),
    L4(Feet.of(6).plus(Inch.of(5)));

    Distance height;

    /**
     * Method to retrieve the height in inches
     * @return the height in inches
     */
    public double getHeightInInches() {
        return height.in(Inch);
    }

}