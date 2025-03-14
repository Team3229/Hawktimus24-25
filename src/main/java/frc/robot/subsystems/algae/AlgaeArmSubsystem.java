package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the algae arm. The arm is used to remove algae from the reef and
 * to collect algae from the ground.
 */
public class AlgaeArmSubsystem extends SubsystemBase {

    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private SparkClosedLoopController positionController;
    private ArmFeedforward feedForward;

    private Angle setpoint;

    private static final int CAN_ID = 9;
    private static final Current CURRENT_LIMIT = Amps.of(40);
    private static final double kP = 0.008;
    private static final double kI = 0;
    private static final double kD = 0.1;

    private static final Angle POSITION_TOLERANCE = Degrees.of(10);
    
    private static final Angle REMOVAL_POSITION = Degrees.of(154); 
    private static final Angle HOME_POSITION = Degrees.of(0);
    private static final Angle THROW_ANGLE = Degrees.of(205);
    private static final Angle GROUND_COLLECT_ANGLE = Degrees.of(90);
    private static final Angle HOLD_ANGLE = Degrees.of(50);
    private static final Angle SCORE_ANGLE = Degrees.of(70);
    private static final Angle FORWARD_LIMIT = Degrees.of(200);
    private static final Angle REVERSE_LIMIT = Degrees.of(0);
    private static final double UP_MAX_SPEED = 0.1;
    private static final double DOWN_MAX_SPEED = 0.1;
    
    private static final double POSITION_CONVERSION_FACTOR = 360;
    private static final double GEARBOX_RATIO = 20;

    public AlgaeArmSubsystem() {
        super();

        armMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();
        positionController = armMotor.getClosedLoopController();
        feedForward = new ArmFeedforward(
            0,
            0.83,
            0.39,
            0.02
        );

        armMotorConfig.inverted(false); 

        armMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        armMotorConfig.idleMode(IdleMode.kBrake);

        armMotorConfig.softLimit 
                .forwardSoftLimitEnabled(false)
                .forwardSoftLimit(FORWARD_LIMIT.in(Degrees))
                .reverseSoftLimitEnabled(false)
                .reverseSoftLimit(REVERSE_LIMIT.in(Degrees));

        armMotorConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(false)
            .pid(
                kP,
                kI,
                kD
            )
            .outputRange(
                -DOWN_MAX_SPEED,
                UP_MAX_SPEED
            );
        
        armMotorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .zeroCentered(false);

        armMotorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR / GEARBOX_RATIO);

        armMotorConfig.closedLoopRampRate(0);

        //ABSOLUTE ENCODER IMPLEMENT

        armMotor.configure(
            armMotorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );
    }

    public Angle getPosition() {
        return Degrees.of(armMotor.getAbsoluteEncoder().getPosition());
    }

    public Angle getRelativePosition() {
        return Degrees.of(armMotor.getEncoder().getPosition());
    }

    /**
     * Command that rotates the arm to a position where the algae can
     * be removed from the reef
     * 
     * @return
     */
    public Command upToRemoveAlgaeFromReef() {
        return rotateArm(REMOVAL_POSITION);
    }

    /**
     * Command to rotate the arm back to its home position
     * 
     * @return
     */
    public Command home() {
        return rotateArm(HOME_POSITION);
    }

    /**
     * @return Command to rotate the arm to a position where algae can be thrown off
     *         the arm across the robot
     *         (i.e. the algae is thrown off the arm and not into the robot)
     */
    public Command upperAlgaeRemovalPosition() {
        return rotateArm(THROW_ANGLE);
    }

    /**
     * Command to rotate the arm to a position where algae can be collected from the
     * ground
     * 
     * @return
     */
    public Command readyingCollectPosition() {
        return rotateArm(GROUND_COLLECT_ANGLE);
    }

    /**
     * * Command to rotate the arm to a position where algae can be scored
     */
    public Command scorePosition() {
        return rotateArm(SCORE_ANGLE);
    }

    /**
     * Command rotates arm to a position where algae can be held
     * 
     * @return
     */
    public Command intakePosition() {
        return rotateArm(HOLD_ANGLE);
    }

    /**
     * Command that rotates the arm to a given position
     * 
     * @param degrees
     * @return
     */
    protected Command rotateArm(Angle angle) {

        Angle desiredAngle = angle;
        Command armRemovalCommand = new Command() {
            @Override
            public void execute() {
                setSetpoint(desiredAngle);
            }

            @Override
            public boolean isFinished() {

                System.out.println(getPosition().minus(desiredAngle).in(Degrees));

                return Math.abs(getPosition().minus(desiredAngle).in(Degrees))
                 < POSITION_TOLERANCE.in(Degrees);
            }
        };

        armRemovalCommand.addRequirements(this);

        return armRemovalCommand;
    }

    public Command disableAlgaeArm() {
        return Commands.runOnce(
            () -> {
                positionController.setReference(0, ControlType.kDutyCycle);
                setpoint = null;
            }
        );
    }

    private void setSetpoint(Angle angle) {

        setpoint = wrapTo360(angle);

        if(RobotBase.isSimulation()) {
            armMotor.getEncoder().setPosition((setpoint.in(Degrees) - armMotor.getEncoder().getPosition()) * kP);
        }

    }

    @Override
    public void periodic() {

        if (setpoint != null) {
            if (getPosition().in(Degrees) > 300) {
                positionController.setReference(wrapTo360(setpoint).plus(Degrees.of(360)).in(Degrees), ControlType.kPosition);
            } else {
                positionController.setReference(wrapTo360(setpoint).in(Degrees), ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward.calculate(MathUtil.inputModulus(getPosition().minus(Degrees.of(90)).in(Radians), 0, 2*Math.PI), 0));
            }
        } else {
            positionController.setReference(0, ControlType.kDutyCycle);
        }
    }

    private Angle wrapTo360(Angle angle) {

        if (MathUtil.inputModulus(angle.in(Degrees), 0, 360) == 360) {
            return Degrees.of(0);
        }

        return Degrees.of(MathUtil.inputModulus(angle.in(Degrees), 0, 360));
    }

    public double getDraw() {
        return armMotor.getOutputCurrent();
    }
}
