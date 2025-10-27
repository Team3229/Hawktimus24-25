package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
 
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ReefHeight;

/**
 * The ElevatorSubsystem class represents the elevator mechanism of the robot.
 * It controls the movement of the elevator using a motor and various sensors.
 */
public class ElevatorSubsystem extends SubsystemBase {

    // CAN ID for the motor controller
    private static final int MOTOR_CAN_ID = 7;
    
    // Maximum speed of the elevator motor
    private static final double UP_MAX_SPEED = 1;
    private static final double DOWN_MAX_SPEED = 0.4;
    
    // Current limit for the motor controller
    private static final Current CURRENT_LIMIT = Amps.of(80);
    
    // Tolerance for the elevator position
    private static final Distance POSITION_TOLERANCE = Inch.of(1);

    private static final LinearVelocity VELOCITY_TOLERANCE = InchesPerSecond.of(2);

    // Base height of the elevator
    public static final Distance ELEVATOR_BASE_HEIGHT = Inch.of(35.75);
    
    // Radius of the sprocket used in the elevator mechanism
    private static final Distance SPROCKET_RADIUS = Inch.of(0.7);
    
    // Circumference of the sprocket
    private static final Distance SPROCKET_CIRCUMFERENCE = SPROCKET_RADIUS.times(2 * Math.PI);

    // Maximum height the elevator can reach
    private static final Distance MAX_ELEVATOR_HEIGHT = Inch.of(42.25);

    // Conversion factor for the elevator position, considering the 2-stage nature of the Swyft Elevator
    private static final double POSITION_CONVERSION_FACTOR = SPROCKET_CIRCUMFERENCE.in(Inch) / 2;

    // PID controller constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 2;

    public static final Time L4_EXTRA_WAIT_TIME = Seconds.of(0.25);

    private SparkMax elevatorMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController positionController;

    private PIDController toleranceController = new PIDController(kP, kI, kD);

    private Distance elevatorHeight;

    public ElevatorSubsystem() {

        super();

        elevatorMotor = new SparkMax(MOTOR_CAN_ID, MotorType.kBrushless);

        positionController = elevatorMotor.getClosedLoopController();

        motorConfig = new SparkMaxConfig();

        motorConfig.inverted(true);

        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);
        
        motorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        motorConfig.idleMode(IdleMode.kBrake);

        motorConfig.softLimit
            .forwardSoftLimit(MAX_ELEVATOR_HEIGHT.in(Inch) * POSITION_CONVERSION_FACTOR / 2)
            .reverseSoftLimit(0)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimitEnabled(true);

        motorConfig.closedLoop
            .pid(kP, kI, kD)
            .outputRange(
                -DOWN_MAX_SPEED,
                UP_MAX_SPEED
            );
        
        elevatorMotor.configure(
            motorConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        elevatorHeight = Inch.of(0);

        toleranceController.setTolerance(POSITION_TOLERANCE.in(Inch), VELOCITY_TOLERANCE.in(InchesPerSecond));
        
    }

    public Command goToLevel(ReefHeight reefPosition) {
        
        Command goToLevelCommand = new Command() {
            @Override
            public void execute() {
                setSetpoint(reefPosition);
            }

            @Override
            public boolean isFinished() {
                // return getElevatorPose().minus(getElevatorRelativeHeight(reefPosition)).abs(Inch) < POSITION_TOLERANCE.in(Inch);
                return atSetpoint();
            }
        };

        goToLevelCommand.addRequirements(this);
        
        return goToLevelCommand;
    }

    public Distance getElevatorRelativeHeight(ReefHeight reefPosition) {
        return reefPosition.getHeight().minus(ELEVATOR_BASE_HEIGHT);
    }

    public void disableElevator() {
        positionController.setReference(0, ControlType.kDutyCycle);
    }

    private void setSetpoint(ReefHeight reefPosition) {

        if (RobotBase.isSimulation()) {
            elevatorHeight = elevatorHeight.plus(getElevatorRelativeHeight(reefPosition).minus(getElevatorPose()).times(0.2));
        }
        
        positionController.setReference(getElevatorRelativeHeight(reefPosition).in(Inch), ControlType.kPosition);
        toleranceController.setSetpoint(getElevatorRelativeHeight(reefPosition).in(Inch));

    }

    public Distance getElevatorPose() {

        if (RobotBase.isSimulation()) {
            return elevatorHeight;
        }

        return Inch.of(elevatorMotor.getEncoder().getPosition());
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Elevator Position", () -> elevatorMotor.getEncoder().getPosition(), null);
    }

    private boolean atSetpoint() {
        toleranceController.calculate(getElevatorPose().in(Inch));
        return toleranceController.atSetpoint();
    }

}
