package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax climbMotor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 16;
    
    private static final double POSITION_CONVERSION_FACTOR = 1; //Change this value to the correct conversion factor later.
    private static final double FORWARD_SOFT_LIMIT = 1; //Change this value to the correct forward soft limit later.
    private static final double REVERSE_SOFT_LIMIT = 1; //Change this value to the correct reverse soft limit later.
    
    public ClimbSubsystem() {

        super();

        climbMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        
        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);
        
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)

            /*Change the value of 1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .forwardSoftLimit(FORWARD_SOFT_LIMIT * POSITION_CONVERSION_FACTOR)
            .reverseSoftLimitEnabled(true)

            /*Change the value of 1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .reverseSoftLimit(REVERSE_SOFT_LIMIT * POSITION_CONVERSION_FACTOR);
    }

    public double getPosition() {
        return climbMotor.getEncoder().getPosition();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Position", () -> climbMotor.getEncoder().getPosition(), null);
    }
}  