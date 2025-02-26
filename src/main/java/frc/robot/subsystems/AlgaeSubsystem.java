package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AlgaeSubsystem extends SubsystemBase {

    private SparkMax armMotor;
    private SparkMaxConfig armMotorConfig;
    private SparkClosedLoopController positionController;
    private SparkMax wheelMotor;
    private SparkMaxConfig wheelMotorConfig;
    private static final Angle POSITION_TOLERANCE = Degrees.of(3);
    private static final double ARM_REMOVAL_POSITION = Degrees.of(135).in(Rotations);// ( ━☞´◔‿◔`)━☞ Replace with actual
                                                                                     // position pls
    private static final double ARM_HOME_POSITION = 0;// ( ━☞´◔‿◔`)━☞ Replace with actual position pls
    private static final int CW_WHEEL_SPEED = 1;
    private static final int CCW_WHEEL_SPEED = -1;
    private static final double ALGAE_THROW_ANGLE = Degrees.of(200).in(Rotations);
    private static final double ALGAE_GROUND_COLLECT_ANGLE = Degrees.of(90).in(Rotations);
    private static final double ALGAE_HOLD_ANGLE = Degrees.of(45).in(Rotations);
    private static final double ALGAE_SCORE_ANGLE = Degrees.of(70).in(Rotations);
    private static final int ARM_MOTOR_CAN_ID = -1; // ( ━☞´◔‿◔`)━☞ Replace with actual CAN ID pls
    private static final int WHEEL_MOTOR_CAN_ID = -1; // ( ━☞´◔‿◔`)━☞ Replace with actual CAN ID pls
    private static final int algaeForwardSoftLimit = 0;
    private static final int algaeReverseSoftLimit = 0;
    private static final int algaeSmartCurrentLimit = 80; // ( ━☞´◔‿◔`)━☞ Replace with actual current limit pls

    public AlgaeSubsystem() {

        armMotor = new SparkMax(ARM_MOTOR_CAN_ID, MotorType.kBrushless);
        wheelMotor = new SparkMax(WHEEL_MOTOR_CAN_ID, MotorType.kBrushless);

        armMotorConfig = new SparkMaxConfig();

        armMotorConfig.inverted(true); // ( ━☞´◔‿◔`)━☞ Replace with actual inversion pls

        armMotorConfig.smartCurrentLimit(algaeSmartCurrentLimit);
        armMotorConfig.idleMode(IdleMode.kBrake);

        armMotorConfig.softLimit // ( ━☞´◔‿◔`)━☞ Replace with actual soft limit pls
                .forwardSoftLimitEnabled(algaeForwardSoftLimit > 0)
                .forwardSoftLimit(algaeForwardSoftLimit)
                .reverseSoftLimitEnabled(algaeReverseSoftLimit > 0)
                .reverseSoftLimit(algaeReverseSoftLimit);

        armMotor.configure(
                armMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        wheelMotorConfig = new SparkMaxConfig();

        wheelMotorConfig.inverted(true); // ( ━☞´◔‿◔`)━☞ Replace with actual inversion pls

        wheelMotorConfig.smartCurrentLimit(algaeSmartCurrentLimit);
        wheelMotorConfig.idleMode(IdleMode.kBrake);

        wheelMotorConfig.softLimit // ( ━☞´◔‿◔`)━☞ Replace with actual soft limit pls
                .forwardSoftLimitEnabled(algaeForwardSoftLimit > 0)
                .forwardSoftLimit(algaeForwardSoftLimit)
                .reverseSoftLimitEnabled(algaeReverseSoftLimit > 0)
                .reverseSoftLimit(algaeReverseSoftLimit);

        wheelMotor.configure(
                wheelMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    /**
     * Command to remove algae from the lower part of the reef, then drop it
     * 
     * @return
     */
    public Command removeLowerAlgae() {
        return armUpToRemoveAlgaeFromReef()
                .alongWith(spinWheel(true))
                .andThen(new WaitCommand(2))
                .andThen(stopWheel())
                .alongWith(armHome());
    }

    /**
     * Command to remove algae from the upper part of the reef, then throws it off
     * the back of the robot
     * 
     * @return Command to remove algae from the upper part of the reef
     */
    public Command removeUpperAlgae() {
        return armUpToRemoveAlgaeFromReef()
                .alongWith(spinWheel(false))
                .andThen(new WaitCommand(1))
                .andThen(upperAlgaeRemmovalPosition())
                .andThen(stopWheel())
                .alongWith(armHome());
    }

    /**
     * Command to get ready to collect algae from the ground
     * 
     * @return Command to collect algae
     */
    public Command readyAlgaeCollection() {
        return readyingCollectPosition()
                .alongWith(spinWheel(true));
    }

    /**
     * Command to hold algae in the arm
     * 
     * @return Command to hold algae
     */
    public Command holdAlgae() {
        return holdPosition()
                .andThen(stopWheel());
    }

    /**
     * Command to score algae
     * 
     * @return Command to score algae
     */
    public Command scoreAlgae() {
        return scorePosition()
                .alongWith(spinWheel(false))
                .andThen(new WaitCommand(2))
                .andThen(stopWheel())
                .alongWith(armHome());
    }

    /*
     * Can you please check if this is needed Nathan? -Reilly
     */
    // public void setMotorSpeed(double speed) {
    // armMotor.set(speed);
    // }

    // public void stopMotor() {
    // armMotor.stopMotor();

    // }

    /**
     * Command that rotates the arm to a position where the algae can
     * be removed from the reef
     * 
     * @return
     */
    public Command armUpToRemoveAlgaeFromReef() {
        return rotateArm(ARM_REMOVAL_POSITION);
    }

    /**
     * Command to rotate the arm back to its home position
     * 
     * @return
     */
    public Command armHome() {
        return rotateArm(ARM_HOME_POSITION);
    }

    /**
     * @return Command to rotate the arm to a position where algae can be thrown off
     *         the arm across the robot
     *         (i.e. the algae is thrown off the arm and not into the robot)
     */
    public Command upperAlgaeRemmovalPosition() {
        return rotateArm(ALGAE_THROW_ANGLE);
    }

    /**
     * Command to rotate the arm to a position where algae can be collected from the
     * ground
     * 
     * @return
     */
    public Command readyingCollectPosition() {
        return rotateArm(ALGAE_GROUND_COLLECT_ANGLE);
    }

    /**
     * * Command to rotate the arm to a position where algae can be scored
     */
    public Command scorePosition() {
        return rotateArm(ALGAE_SCORE_ANGLE);
    }

    /**
     * Command rotates arm to a position where algae can be held
     * 
     * @return
     */
    public Command holdPosition() {
        return rotateArm(ALGAE_HOLD_ANGLE);
    }

    /**
     * Command that rotates the arm to a given position
     * 
     * @param rotations
     * @return
     */
    protected Command rotateArm(double rotations) {

        Command armRemovalCommand = new Command() {
            @Override
            public void initialize() {
                setSetpoint(rotations);
            }

            @Override
            public boolean isFinished() {
                return Math.abs(armMotor.getEncoder().getPosition()
                        - rotations) < POSITION_TOLERANCE.in(Rotations);
            }
        };

        armRemovalCommand.addRequirements(this);

        return armRemovalCommand;
    }

    /**
     * Spins the algae wheel in a given direction
     * 
     * @param clockwise true if clockwise, false if counterclockwise
     * @return Command to spin the algae wheel
     */
    public Command spinWheel(boolean clockwise) {

        Command spinWheelCommand = new Command() {
            @Override
            public void initialize() {
                setWheelDirection(clockwise);
            }

        };

        spinWheelCommand.addRequirements(this);

        return spinWheelCommand;
    }

    /**
     * Command to stop the algae wheel
     * 
     * @return Command to stop the algae wheel
     */
    public Command stopWheel() {
        Command stopWheelCommand = new Command() {
            @Override
            public void initialize() {
                wheelMotor.stopMotor();
            }
        };

        stopWheelCommand.addRequirements(this);

        return stopWheelCommand;
    }

    private void setSetpoint(double rotations) {

        positionController.setReference(rotations, ControlType.kPosition);

    }

    private void setWheelDirection(Boolean clockwise) {

        if (clockwise) {
            wheelMotor.set(CW_WHEEL_SPEED);
        } else {
            wheelMotor.set(CCW_WHEEL_SPEED);
        }

    }
}
