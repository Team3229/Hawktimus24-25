package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climbMotor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 10;
    private static final double POSITION_CONVERSION_FACTOR = 1; //Change this value to the correct conversion factor later.
    
    public ClimbSubsystem(DoubleSupplier joystickInput) {
        climbMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        
        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);
        
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)

            /*Change the value of 1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .forwardSoftLimit(1 * POSITION_CONVERSION_FACTOR)
            .reverseSoftLimitEnabled(true)

            /*Change the value of -1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .reverseSoftLimit(1 * POSITION_CONVERSION_FACTOR);

        setDefaultCommand(
            Commands.run(
                () -> {
                    climbMotor.set(joystickInput.getAsDouble());
                }, this)
        );
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Position", () -> climbMotor.getEncoder().getPosition(), null);
    }
}  