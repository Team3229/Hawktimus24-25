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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
  
    private SparkMax climbMotor;
    private SparkMax climbMotor2;
    private SparkMaxConfig motorConfig;
    private SparkMaxConfig motorConfig2;
    private Servo servo;
    private Servo servo2;

    private ClimbCamera camera;

    private static final int CAN_ID = 8;
    private static final int CAN_ID_2 = 4;
    private static final int PWM_ID = 8; // replace with actual PWM ID
    private static final int PWM_ID_2 = 7; // replace with actual PWM ID
    
    private static final double POSITION_CONVERSION_FACTOR = 360;
    private static final double GEARBOX_RATIO = 125;
    private static final double ENGAGED_SERVO_ANGLE = 150;
    private static final double DISENGAGED_SERVO_ANGLE = 10;
    private static final double ENGAGED_SERVO2_ANGLE = 135;
    private static final double DISENGAGED_SERVO2_ANGLE = 90;

    private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(50);
    private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-110);

    public static final Time AUTOLOCK_BEFORE_MATCH_END = Seconds.of(1);
    public static final boolean AUTOLOCK_ENABLED = true;

    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;
    private static final int RAMP_RATE = 1;
    
    private static final double CLIMB_SPEED = 0.6;

    private boolean isServoEngaged = false;
    
    public ClimbSubsystem() {

        super();

        climbMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        climbMotor2 = new SparkMax(CAN_ID_2, MotorType.kBrushless);
        servo = new Servo(PWM_ID);
        servo2 = new Servo(PWM_ID_2);

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

        camera = new ClimbCamera();

        seedInternalEncoder();

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
        climbMotor.set(speed * CLIMB_SPEED);
    }

    public void stop() {
        climbMotor.stopMotor();
    }

    public Command engageServoCommand() {
        return runOnce(
            this::engageServo
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
                if (isServoEngaged) {
                    disengageServo();
                } else {
                    engageServo();
                }
            }
        );
    }

    public void engageServo() {
        servo.setAngle(ENGAGED_SERVO_ANGLE);
        servo2.setAngle(ENGAGED_SERVO2_ANGLE);
        isServoEngaged = true;

    }

    public void disengageServo() {
        servo.setAngle(DISENGAGED_SERVO_ANGLE);
        servo2.setAngle(DISENGAGED_SERVO2_ANGLE);
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

    private void seedInternalEncoder() {
        climbMotor.getEncoder().setPosition(getCurrentAngle().in(Degrees));
        climbMotor2.getEncoder().setPosition(getCurrentAngle().in(Degrees));
    }

    public UsbCamera getCamera() {
        return camera.camera;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Position", () -> getCurrentAngle().in(Degrees), null);
        builder.addDoubleProperty("Relative Climb Pose", this.climbMotor.getEncoder()::getPosition, null);
        builder.addBooleanProperty("Is Climb Engaged", () -> isServoEngaged, null);
    }
}  