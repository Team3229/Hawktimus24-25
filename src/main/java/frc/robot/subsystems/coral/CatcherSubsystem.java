package frc.robot.subsystems.coral;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CatcherSubsystem extends SubsystemBase{

    private DigitalInput irSensor;
    private DigitalInput limitSwitch;
    private SparkMax catcherMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController positionController;

    //25 degrees in cad (NEEDS TO BE TESTED)
    private static final Angle FEED_ANGLE = Degrees.of(45);
    private static final Angle CATCH_ANGLE = Degrees.of(0);

    private static final double CONVERSION_FACTOR = (1.0/5.0)*360.0;
    
    private static final int CAN_ID = 5;
    private static final int SENSOR_PORT = 9;
    private static final int LIMIT_PORT = 7;
    
    private static final double MAX_SPEED = 0.1;
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;

    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;

    
    public CatcherSubsystem() {

        super();

        catcherMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        positionController = catcherMotor.getClosedLoopController();
        
        irSensor = new DigitalInput(SENSOR_PORT);

        limitSwitch = new DigitalInput(LIMIT_PORT);
        
        motorConfig.idleMode(IDLE_MODE);

        motorConfig.closedLoop
            .pid(
                kP,
                kI,
                kD
            )
            .outputRange(
                -MAX_SPEED,
                MAX_SPEED
            );

        motorConfig.encoder
            .positionConversionFactor(CONVERSION_FACTOR);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(FEED_ANGLE.in(Degrees))
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(CATCH_ANGLE.in(Degrees));

        motorConfig.smartCurrentLimit(40);
            
        catcherMotor.configure(
            motorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        limit().onTrue(
            Commands.runOnce(this::zeroFeeder, this).ignoringDisable(true)
        );

        hasCoral().onFalse(
            catchAngle()
        );
        
    }

    public Command feedAngle() {

        return Commands.runOnce(
            () -> {
                System.out.println("Feed");
                positionController.setReference(FEED_ANGLE.in(Degrees), ControlType.kPosition);
            },
            this
        ).handleInterrupt(() -> positionController.setReference(0, ControlType.kDutyCycle));
    }
    
    public Command catchAngle() {
        return Commands.runOnce(
            () -> {
                System.out.println("Reset Catcher");
                positionController.setReference(0, ControlType.kDutyCycle);
            },
            this
        );
    }

    public Angle getAngle() {
        return Degrees.of(catcherMotor.getEncoder().getPosition());
    }

    public Trigger hasCoral() {
        return new Trigger(this::sensor);
    }

    private boolean sensor() {
        return !irSensor.get();
    }
    
    public Trigger limit() {
        return new Trigger(this::limitSwitch);
    }

    private boolean limitSwitch() {
        return limitSwitch.get();
    }

    private void zeroFeeder() {
        catcherMotor.getEncoder().setPosition(0);
    }

    public double getDraw() {
        return catcherMotor.getOutputCurrent();
    }

}