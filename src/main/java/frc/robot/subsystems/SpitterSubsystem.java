package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;

public class SpitterSubsystem {
    
    private SparkMax spitterMotor;
    private DigitalInput spitterSensor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 3;
    private static final int SENSOR_PORT = 0;

    public SpitterSubsystem() {
        spitterMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        spitterSensor = new DigitalInput(SENSOR_PORT);
    }

    public Command spit() {
        return new Command() {
            @Override
            public void execute() {
                spitterMotor.set(-0.5);
            }

            @Override
            public boolean isFinished() {
                return spitterSensor.get();
            }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
               
                motorConfig.smartCurrentLimit(38); //change to actual desired current limit later
            }
        
        };
    }

}
