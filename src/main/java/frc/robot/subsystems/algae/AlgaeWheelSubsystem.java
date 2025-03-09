package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the algae wheel. 
 */
public class AlgaeWheelSubsystem extends SubsystemBase {

    private SparkMax wheelMotor;
    private SparkMaxConfig wheelMotorConfig;

    private static final int CAN_ID = 18;
    
    private static final boolean INVERTED = true;
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;

    private static final int CW_SPEED = 1;
    private static final int CCW_SPEED = -1;
    private static final int CURRENT_LIMIT = 1;

    public AlgaeWheelSubsystem() {

        wheelMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        wheelMotorConfig = new SparkMaxConfig();

        wheelMotorConfig.inverted(INVERTED);

        wheelMotorConfig.smartCurrentLimit(CURRENT_LIMIT);
        wheelMotorConfig.idleMode(IDLE_MODE);

        wheelMotor.configure(
                wheelMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        
    }

    /**
     * Spins the algae wheel in a given direction
     * 
     * @param clockwise true if clockwise, false if counterclockwise
     * @return Command to spin the algae wheel
     */
    public Command spin(boolean clockwise) {
        return Commands.runOnce(
            () -> setWheelDirection(clockwise)
        );
    }

    /**
     * Command to stop the algae wheel
     * 
     * @return Command to stop the algae wheel
     */
    public Command stop() {
        return Commands.runOnce(
            () -> stopWheel()
        );
    };

    protected void stopWheel() {
        wheelMotor.stopMotor();
    }

    private void setWheelDirection(boolean clockwise) {

        if (clockwise) {
            wheelMotor.set(CW_SPEED);
        } else {
            wheelMotor.set(CCW_SPEED);
        }

    }
    public double getDraw() {
        return wheelMotor.getOutputCurrent();
    }
}
