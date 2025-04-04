package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
  
    private SparkMax climbMotor;
    private SparkMax climbMotor2;
    private SparkMaxConfig motorConfig;
    private SparkMaxConfig motorConfig2;
    private Servo servoRight;
    private Servo servoLeft;

    // private ClimbCamera camera;

    private static final int CAN_ID = 8;
    private static final int CAN_ID_2 = 4;
    private static final int PWM_ID = 8;
    private static final int PWM_ID_2 = 7;
    
    private static final double POSITION_CONVERSION_FACTOR = 360;
    private static final double GEARBOX_RATIO = 125;

    private static final double ENGAGED_SERVO_RIGHT_ANGLE = 135;
    private static final double DISENGAGED_SERVO_RIGHT_ANGLE = 170;

    private static final double ENGAGED_SERVO_LEFT_ANGLE = 115;
    private static final double DISENGAGED_SERVO_LEFT_ANGLE = 85;

    private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(75);
    private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-90);

    public static final Time AUTOLOCK_BEFORE_MATCH_END = Seconds.of(1);
    public static final boolean AUTOLOCK_ENABLED = true;                

    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final int RAMP_RATE = 1;
    
    private static final double CLIMB_SPEED = 0.6;

    private boolean isServoEngaged = false;

    private boolean preventManualLockToggle = false;
    
    public ClimbSubsystem() {

        super();

        climbMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        climbMotor2 = new SparkMax(CAN_ID_2, MotorType.kBrushless);
        servoRight = new Servo(PWM_ID);
        servoLeft = new Servo(PWM_ID_2);

        motorConfig = getBaseConfig();

        motorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)
            .forwardSoftLimit(FORWARD_SOFT_LIMIT.in(Degrees))
            .reverseSoftLimitEnabled(true)
            .reverseSoftLimit(REVERSE_SOFT_LIMIT.in(Degrees));

        motorConfig.absoluteEncoder
            .zeroCentered(true);

        climbMotor.configure(
            motorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        motorConfig2 = getBaseConfig();

        motorConfig2.follow(
            climbMotor,
            true
        );

        climbMotor2.configure(
            motorConfig2,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // camera = new ClimbCamera();

        seedInternalEncoder();

        servoRight.setAngle(DISENGAGED_SERVO_RIGHT_ANGLE);  
        servoLeft.setAngle(DISENGAGED_SERVO_LEFT_ANGLE);

        SmartDashboard.putBoolean("ENABLE_AUTOLOCK", true);

    }

    private SparkMaxConfig getBaseConfig() {
        SparkMaxConfig config = new SparkMaxConfig();

        config.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR / GEARBOX_RATIO);
        
        config.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        config.idleMode(IDLE_MODE);

        config.openLoopRampRate(RAMP_RATE);

        return config;
    }

    public Angle getCurrentAngle() {
        return Degrees.of(climbMotor.getAbsoluteEncoder().getPosition());
    }

    public void setSpeed(double speed) {
        // System.out.println("Climbing at speed " + speed);
        climbMotor.setVoltage(speed * CLIMB_SPEED * 12);
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    public Command engageServoCommand() {
        return runOnce(
            this::engageServo
        );
    }

    public Command forceEngageCommand() {
        return runOnce(
            () -> {
                if (SmartDashboard.getBoolean("ENABLE_AUTOLOCK", true)) {
                    preventManualLockToggle = true;
                    engageServo();
                }
            }
        );
    }

    public Command disengageServoCommand() {
        return runOnce(
            this::disengageServo
        );
    }

    public Command toggleServo() {
        return runOnce(
            () -> {

                if (preventManualLockToggle) {
                    return;
                }

                if (isServoEngaged) {
                    disengageServo();
                } else {
                    engageServo();
                }
            }
        );
    }

    public void engageServo() {
        servoRight.setAngle(ENGAGED_SERVO_RIGHT_ANGLE);
        servoLeft.setAngle(ENGAGED_SERVO_LEFT_ANGLE);
        isServoEngaged = true;

    }

    public void disengageServo() {
        servoRight.setAngle(DISENGAGED_SERVO_RIGHT_ANGLE);
        servoLeft.setAngle(DISENGAGED_SERVO_LEFT_ANGLE);
        isServoEngaged = false;
    }

    public boolean isServoEngaged() {
        return isServoEngaged;
    }

    public Command engageClimb() {
        return runEnd(() -> {
            setSpeed(-CLIMB_SPEED);
        },
        () -> {
            stop();
        });
    }

    public Command disengageClimb() {
        return runEnd(() -> {
            setSpeed(CLIMB_SPEED);
        },
        () -> {
            stop();
        });
    }

    public void seedInternalEncoder() {
        climbMotor.getEncoder().setPosition(getCurrentAngle().in(Degrees));
        climbMotor2.getEncoder().setPosition(getCurrentAngle().in(Degrees));
    }

    // public UsbCamera getCamera() {
    //     return camera.camera;
    // }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Position", () -> getCurrentAngle().in(Degrees), null);
        builder.addDoubleProperty("Relative Climb Pose", this.climbMotor.getEncoder()::getPosition, null);
        builder.addBooleanProperty("Is Climb Engaged", () -> isServoEngaged, null);
    }
}  