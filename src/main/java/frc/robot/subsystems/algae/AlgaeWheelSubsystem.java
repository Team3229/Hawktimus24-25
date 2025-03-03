package frc.robot.subsystems.algae;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
    private static final int CW_WHEEL_SPEED = 1;
    private static final int CCW_WHEEL_SPEED = -1;
    private static final int WHEEL_MOTOR_CAN_ID = 19; // ( ━☞´◔‿◔`)━☞ Replace with actual CAN ID pls
    private static final int algaeForwardSoftLimit = 0;
    private static final int algaeReverseSoftLimit = 0;
    private static final int algaeSmartCurrentLimit = 80; // ( ━☞´◔‿◔`)━☞ Replace with actual current limit pls

    public AlgaeWheelSubsystem() {
        wheelMotor = new SparkMax(WHEEL_MOTOR_CAN_ID, MotorType.kBrushless);
        wheelMotorConfig = new SparkMaxConfig();

        wheelMotorConfig.inverted(true); // ( ━☞´◔‿◔`)━☞ Replace with actual inversion pls

        wheelMotorConfig.smartCurrentLimit(algaeSmartCurrentLimit);
        wheelMotorConfig.idleMode(IdleMode.kBrake);
        wheelMotorConfig.softLimit // ( ━☞´◔‿◔`)━☞ Replace with actual soft limit pls
                .forwardSoftLimitEnabled(algaeForwardSoftLimit > 0)
                .forwardSoftLimit(algaeForwardSoftLimit)
                .reverseSoftLimitEnabled(algaeReverseSoftLimit > 0)
                .reverseSoftLimit(algaeReverseSoftLimit);

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
            () -> wheelMotor.stopMotor()
        );
    };

    private void setWheelDirection(boolean clockwise) {

        if (clockwise) {
            wheelMotor.set(CW_WHEEL_SPEED);
        } else {
            wheelMotor.set(CCW_WHEEL_SPEED);
        }

    }
}
