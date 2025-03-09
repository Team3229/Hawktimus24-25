package frc.robot.subsystems.algae;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Subsystem for the algae arm. The arm is used to remove algae from the reef and
 * to collect algae from the ground.
 */
public class AlgaeArmSubsystem extends SubsystemBase {

    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private SparkClosedLoopController positionController;

    private static final int CAN_ID = 9; 
    private static final Current CURRENT_LIMIT = Amps.of(1); // ( ━☞´◔‿◔`)━☞ Replace with actual current limit pls
    private static final double kP = 0.1;
    private static final double kI = 0;
    private static final double kD = 0;
    private static final Angle ENCODER_OFFSET = Degrees.of(304);

    private static final Angle POSITION_TOLERANCE = Degrees.of(3);
    
    private static final Angle REMOVAL_POSITION = Degrees.of(135); // ( ━☞´◔‿◔`)━☞ Replace with actual position pls
    private static final Angle HOME_POSITION = Degrees.of(0); // ( ━☞´◔‿◔`)━☞ Replace with actual position pls
    private static final Angle THROW_ANGLE = Degrees.of(200);
    private static final Angle GROUND_COLLECT_ANGLE = Degrees.of(90);
    private static final Angle HOLD_ANGLE = Degrees.of(45);
    private static final Angle SCORE_ANGLE = Degrees.of(70);
    private static final Angle FORWARD_LIMIT = Degrees.of(0);
    private static final Angle REVERSE_LIMIT = Degrees.of(0);
    
    private static final double POSITION_CONVERSION_FACTOR = 360;
    private static final double GEARBOX_RATIO = 20;

    public AlgaeArmSubsystem() {

        armMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        armMotorConfig = new SparkMaxConfig();

        armMotorConfig.inverted(false); // ( ━☞´◔‿◔`)━☞ Replace with actual inversion pls

        armMotorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));
        armMotorConfig.idleMode(IdleMode.kBrake);

        armMotorConfig.softLimit // ( ━☞´◔‿◔`)━☞ Replace with actual soft limit pls
                .forwardSoftLimitEnabled(FORWARD_LIMIT.in(Degrees) > 0)
                .forwardSoftLimit(FORWARD_LIMIT.in(Degrees))
                .reverseSoftLimitEnabled(REVERSE_LIMIT.in(Degrees) > 0)
                .reverseSoftLimit(REVERSE_LIMIT.in(Degrees));

        armMotorConfig.closedLoop
            .pid(
                kP,
                kI,
                kD
            );
        
        armMotorConfig.absoluteEncoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR);

        armMotorConfig.encoder
            .positionConversionFactor(POSITION_CONVERSION_FACTOR / GEARBOX_RATIO);


        //ABSOLUTE ENCODER IMPLEMENT

        armMotor.configure(
                armMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        positionController = armMotor.getClosedLoopController();

        seedInternalEncoder();
    }

    public Angle getPosition() {
        return Degrees.of(getAbsoluteEncoderPosition()).minus(ENCODER_OFFSET);
    }

    public double getAbsoluteEncoderPosition() {
        return armMotor.getAbsoluteEncoder().getPosition();
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
     * @param rotations
     * @return
     */
    protected Command rotateArm(Angle angle) {

        Command armRemovalCommand = new Command() {
            @Override
            public void initialize() {
                setSetpoint(angle);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(armMotor.getEncoder().getPosition()
                - angle.in(Rotations)) < POSITION_TOLERANCE.in(Rotations);
            }
        };

        armRemovalCommand.addRequirements(this);

        return armRemovalCommand;
    }

    private void setSetpoint(Angle angle) {

        if(RobotBase.isSimulation()) {
            armMotor.getEncoder().setPosition((angle.in(Rotations) - armMotor.getEncoder().getPosition()) * kP);
        }

        positionController.setReference(angle.in(Rotations), ControlType.kPosition);

    }
    public double getDraw() {
        return armMotor.getOutputCurrent();
    }

    private void seedInternalEncoder() {
        armMotor.getEncoder().setPosition(getPosition().in(Degrees));
    }
}
