package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {
    
    private SparkMax algaeMotor;
    private SparkMaxConfig motorConfig;

    private static final int MOTOR_CAN_ID = -1; // ( ━☞´◔‿◔`)━☞ Replace with actual CAN ID pls

    public AlgaeSubsystem() {

        algaeMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(true); // ( ━☞´◔‿◔`)━☞ Replace with actual inversion pls
        
        motorConfig.smartCurrentLimit(80);
        motorConfig.idleMode(IdleMode.kBrake);
        
        motorConfig.softLimit // ( ━☞´◔‿◔`)━☞ Replace with actual soft limit pls
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(0)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0);
            
    }

    public void setMotorSpeed(double speed) {
        algaeMotor.set(speed);
    }

    public void stopMotor() {
        algaeMotor.stopMotor();
        
    }

}

