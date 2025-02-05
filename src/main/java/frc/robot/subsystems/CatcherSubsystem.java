package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CatcherSubsystem extends SubsystemBase{
    private DigitalInput irSensor;
    public Trigger coral;

    //25 degrees in cad (NEEDS TO BE TESTED)
    private static final double FEED_ANGLE = 25;
    private static final double CATCH_ANGLE = 0;
    
    private static final int MOTOR_CAN_ID = 9;
    private static final double MAX_SPEED = 1;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private SparkMax catcherMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController positionController;

    
    public CatcherSubsystem() {
        catcherMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

        positionController = catcherMotor.getClosedLoopController();

        irSensor = new DigitalInput(0);
        
        motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.closedLoop
        .pid(kP, kI, kD)
        .outputRange(
            -MAX_SPEED,
            MAX_SPEED
        );

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(FEED_ANGLE)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(CATCH_ANGLE);       
        
        coral = new Trigger(this::hasCoral);

        coral.onTrue(
                feedAngle()
            .andThen(
                catchAngle()
            )
        );
    }

    public Command feedAngle () {
        return Commands.runOnce(
            () -> {
                positionController.setReference(FEED_ANGLE, ControlType.kPosition);
            }, this );
    }
    
    public Command catchAngle () {
        return Commands.runOnce(
            () -> {
                positionController.setReference(CATCH_ANGLE, ControlType.kPosition);
            }, this );
    }

    private boolean hasCoral() {
        return !irSensor.get();
    }
}