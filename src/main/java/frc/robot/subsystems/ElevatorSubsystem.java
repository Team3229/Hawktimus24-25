package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;

import java.util.function.DoubleSupplier;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ReefHeight;

public class ElevatorSubsystem extends SubsystemBase {

    private static final double MAX_SPEED = 1;
    private static final double POSITION_TOLERANCE = 0.5;
    private static final double ELEVATOR_BASE_HEIGHT = 35.368230;
    private static final int MOTOR_CAN_ID = 2;
    private static final Distance SPROCKET_RADIUS = Inch.of(0.7);
    private static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS
        .times(2 * Math.PI);
    // Conversion factor is half the circumference of the sprocket because of the 2-Stage nature of the Swyft Elevator
    private static final double POSITION_CONVERSION_FACTOR = SPROCKET_CIRCUMFERENCE.in(Inch) / 2;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    private SparkMax elevatorMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController positionController;

    public ElevatorSubsystem(DoubleSupplier joystickInput) {

        elevatorMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

        positionController = elevatorMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(true);

        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);
        
        motorConfig.smartCurrentLimit(80);
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(30 * POSITION_CONVERSION_FACTOR / 2)
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(0);

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

        setDefaultCommand(
            Commands.run(
                () -> {
                    elevatorMotor.set(joystickInput.getAsDouble());
                }, this)
        );
        
    }

    public Command goToLevel(ReefHeight reefPosition) {
        return new Command() {
            @Override
            public void initialize() {
                setSetpoint(reefPosition);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(elevatorMotor.getEncoder().getPosition() - getElevatorRelativeHeightInInches(reefPosition)) < POSITION_TOLERANCE;
            }
        };
    }

    private double getElevatorRelativeHeightInInches(ReefHeight reefPosition) {
        return reefPosition.getHeightInInches() - ELEVATOR_BASE_HEIGHT;
    }

    private void setSetpoint(ReefHeight reefPosition) {
        
        positionController.setReference(getElevatorRelativeHeightInInches(reefPosition), ControlType.kPosition);

    }

    public void runManualSpeed(double speed) {
        positionController.setReference(speed, ControlType.kDutyCycle);
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Elevator Position", () -> elevatorMotor.getEncoder().getPosition(), null);
    }

}
