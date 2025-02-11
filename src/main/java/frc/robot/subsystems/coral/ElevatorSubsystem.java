package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Inch;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ReefHeight;

/**
 * The ElevatorSubsystem class represents the elevator mechanism of the robot.
 * It controls the movement of the elevator using a motor and various sensors.
 */
public class ElevatorSubsystem extends SubsystemBase {

    // CAN ID for the motor controller
    private static final int MOTOR_CAN_ID = 2;
    
    // Maximum speed of the elevator motor
    private static final double MAX_SPEED = 1;
    
    // Current limit for the motor controller
    private static final int CURRENT_LIMIT = 80;
    
    // Tolerance for the elevator position
    private static final Distance POSITION_TOLERANCE = Inch.of(1/2);

    // Base height of the elevator
    public static final Distance ELEVATOR_BASE_HEIGHT = Inch.of(35.588230);
    
    // Radius of the sprocket used in the elevator mechanism
    private static final Distance SPROCKET_RADIUS = Inch.of(0.7);
    
    // Circumference of the sprocket
    private static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS.times(2 * Math.PI);

    // Maximum height the elevator can reach
    private static final Distance MAX_ELEVATOR_HEIGHT = Inch.of(30);

    // Conversion factor for the elevator position, considering the 2-stage nature of the Swyft Elevator
    private static final double POSITION_CONVERSION_FACTOR = SPROCKET_CIRCUMFERENCE.in(Inch) / 2;

    // PID controller constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private SparkMax elevatorMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController positionController;

    public ElevatorSubsystem() {

        elevatorMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

        positionController = elevatorMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(true);

        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);
        
        motorConfig.smartCurrentLimit(CURRENT_LIMIT);
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.softLimit
            .forwardSoftLimit(MAX_ELEVATOR_HEIGHT.in(Inch) * POSITION_CONVERSION_FACTOR / 2)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        motorConfig.closedLoop
            .pid(kP, kI, kD)
            .outputRange(
                -MAX_SPEED,
                MAX_SPEED
            );
        
        elevatorMotor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
        
    }

    public Command goToLevel(ReefHeight reefPosition) {
        
        Command goToLevelCommand = new Command() {
            @Override
            public void initialize() {
                setSetpoint(reefPosition);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(elevatorMotor.getEncoder().getPosition() - getElevatorRelativeHeightInInches(reefPosition)) < POSITION_TOLERANCE.in(Inch);
            }
        };

        goToLevelCommand.addRequirements(this);
        
        return goToLevelCommand;
    }

    public double getElevatorRelativeHeightInInches(ReefHeight reefPosition) {
        return reefPosition.getHeightInInches() - ELEVATOR_BASE_HEIGHT.in(Inch);
    }

    private void setSetpoint(ReefHeight reefPosition) {
        
        positionController.setReference(getElevatorRelativeHeightInInches(reefPosition), ControlType.kPosition);

    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Elevator Position", () -> elevatorMotor.getEncoder().getPosition(), null);
    }

}
