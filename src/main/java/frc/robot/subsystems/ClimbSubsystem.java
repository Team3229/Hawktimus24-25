package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
    
    private SparkMax climbMotor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 8;
    
    private static final double POSITION_CONVERSION_FACTOR = 360;
    private static final double GEARBOX_RATIO = 100;

    private static final Angle FORWARD_SOFT_LIMIT = Degrees.of(15);
    private static final Angle REVERSE_SOFT_LIMIT = Degrees.of(-110);

    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final IdleMode IDLE_MODE = IdleMode.kBrake;

    private static final double CLIMB_SPEED = 1;
    
    public ClimbSubsystem() {

        super();

        climbMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        motorConfig = new SparkMaxConfig();
        
        motorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);

        motorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR / GEARBOX_RATIO);
        
        motorConfig.idleMode(IDLE_MODE);

        motorConfig.softLimit
            .forwardSoftLimitEnabled(true)

            /*Change the value of 1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .forwardSoftLimit(FORWARD_SOFT_LIMIT.in(Degrees))
            .reverseSoftLimitEnabled(true)

            /*Change the value of 1 to the correct forward soft limit later,
            and double check that POSITION_CONVERSION_FACTOR is used correctly. */
            .reverseSoftLimit(REVERSE_SOFT_LIMIT.in(Degrees));

        motorConfig.absoluteEncoder
            .zeroCentered(true);

        motorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        motorConfig.inverted(false);

        climbMotor.configure(
            motorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        seedInternalEncoder();

    }

    public Angle getCurrentAngle() {
        return Degrees.of(climbMotor.getAbsoluteEncoder().getPosition());
    }

    public void setSpeed(double speed) {
        System.out.println("Climbing at speed " + speed);
        climbMotor.set(speed * CLIMB_SPEED);
    }

    public void stop() {
        climbMotor.stopMotor();
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
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Climb Position", () -> getCurrentAngle().in(Degrees), null);
        builder.addDoubleProperty("Small", this.climbMotor.getEncoder()::getPosition, null);
    }
}  