package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpitterSubsystem extends SubsystemBase {

    private static final double INTAKE_SPEED = 0.5;
    private static final double SPIT_SPEED = -0.5;
    private SparkMax spitterMotor;
    private DigitalInput spitterSensor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 14;
    private static final int SENSOR_PORT = 0;

    public SpitterSubsystem() {
        super();
        spitterMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        spitterSensor = new DigitalInput(SENSOR_PORT);

        motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(38);

        spitterMotor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    /**
     * Checks if the spitter has coral.
     * 
     * @return True if the spitter has coral, false otherwise.
     */
    protected boolean hasCoral() {
        return !spitterSensor.get();
    }

    /**
     * Intakes the coral until the coral is detected by the sensor.
     * 
     * @return Command that intakes the coral.
     */
    public Command intake() {
        return new Command() {
            @Override
            public void execute() {
                spitterMotor.set(INTAKE_SPEED);
            }

            // @Override
            // public boolean isFinished() {
            //     return hasCoral();
            // }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
            }

        }.withTimeout(2);
    }

    /**
     * Spits out the coral until the coral is not detected by the sensor.
     * 
     * @return Command that spits out the coral.
     */
    public Command spit() {
        return new Command() {
            @Override
            public void execute() {
                spitterMotor.set(SPIT_SPEED);
            }

            // @Override
            // public boolean isFinished() {
            //     return !hasCoral();
            // }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
            }
        }.withTimeout(2);
    }
}
