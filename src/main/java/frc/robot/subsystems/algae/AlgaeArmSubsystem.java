package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeArmSubsystem extends SubsystemBase {

    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private ArmFeedforward feedForward;
    private ProfiledPIDController pidController;

    private static final int CAN_ID = 9;
    private static final double POSITION_CONVERSION_FACTOR = 2 * Math.PI;

    public static final Angle REMOVAL_POSITION = Degrees.of(64);
    public static final Angle HOME_POSITION = Degrees.of(-90);
    public static final Angle THROW_ANGLE = Degrees.of(115);
    public static final Angle GROUND_COLLECT_ANGLE = Degrees.of(-20);
    public static final Angle HOLD_ANGLE = Degrees.of(-50);
    public static final Angle SCORE_ANGLE = Degrees.of(-20);

    private static final double kP = 2.5;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(5);
    private static final AngularAcceleration MAX_ACCELERATION = RadiansPerSecondPerSecond.of(5);

    private static final Angle POSITION_TOLERANCE = Degrees.of(0.1);
    private static final AngularVelocity VELOCITY_TOLERANCE = RadiansPerSecond.of(0.1);

    public AlgaeArmSubsystem() {

        armMotor = new SparkMax(CAN_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        feedForward = new ArmFeedforward(
            0,
            0.83,
            0.39,
            0.02
        );

        pidController = new ProfiledPIDController(
            kP,
            kI,
            kD,
            new Constraints(
                MAX_VELOCITY.in(RadiansPerSecond),
                MAX_ACCELERATION.in(RadiansPerSecondPerSecond)
            )
        );

        pidController.setTolerance(
            POSITION_TOLERANCE.in(Radians),
            VELOCITY_TOLERANCE.in(RadiansPerSecond)
        );

        setSetpoint(HOME_POSITION);
        pidController.reset(getPosition().in(Radians));

        armMotorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(POSITION_CONVERSION_FACTOR)
            .zeroCentered(true);

        armMotor.configure(
            armMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

    }

    public Angle getPosition() {
        return Radians.of(armMotor.getAbsoluteEncoder().getPosition());
    }

    public AngularVelocity getVelocity() {
        return RadiansPerSecond.of(armMotor.getAbsoluteEncoder().getVelocity());
    }

    public Command rotateTo(Angle setpoint) {
        return runOnce(
            () -> setSetpoint(setpoint)
        ).until(
            () -> atGoal()
        );
    }

    private void setSetpoint(Angle setpoint) {
        pidController.setGoal(setpoint.in(Radians));
    }

    private boolean atGoal() {
        return pidController.atGoal();
    }

    @Override
    public void periodic() {
        armMotor.setVoltage(
            pidController.calculate(getPosition().in(Radians)) +
            feedForward.calculate(
                pidController.getSetpoint().position,
                pidController.getSetpoint().velocity
            )
        );
    }

}