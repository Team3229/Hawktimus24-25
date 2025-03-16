package frc.robot.subsystems.coral;

import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SpitterSubsystem extends SubsystemBase {

    private SparkMax spitterMotor;
    private DigitalInput spitterSensor;
    private SparkMaxConfig motorConfig;

    private static final int CAN_ID = 6;
    private static final int SENSOR_PORT = 8;

    private static final double INTAKE_SPEED = 0.3;
    // private static final double SPIT_SPEED = 0.8;
    private static final double SPIT_SPEED = 0.3;
    private static final Current CURRENT_LIMIT = Amps.of(80);
    private static final boolean INVERTED = true;

    public SpitterSubsystem() {

        super();
        spitterMotor = new SparkMax(CAN_ID, MotorType.kBrushless);
        spitterSensor = new DigitalInput(SENSOR_PORT);

        motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit((int) CURRENT_LIMIT.in(Amps));

        motorConfig.inverted(INVERTED);

        motorConfig.idleMode(IdleMode.kBrake);

        spitterMotor.configure(
            motorConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

    }

    /**
     * Checks if the spitter has coral.
     * 
     * @return True if the spitter has coral, false otherwise.
     */
    public Trigger hasCoral() {
        return new Trigger(this::sensor);
    }

    private boolean sensor() {
        return !spitterSensor.get();
    }

    /**
     * Intakes the coral until the coral is detected by the sensor.
     * 
     * @return Command that intakes the coral.
     */
    public Command intake() {
        return new Command() {

            @Override
            public void initialize() {
                // System.out.println("Intake");
            }

            @Override
            public void execute() {
                spitterMotor.set(INTAKE_SPEED);
            }

            @Override
            public boolean isFinished() {
                return hasCoral().getAsBoolean();
            }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
            }

        };
    }

    /**
     * Spits out the coral until the coral is not detected by the sensor.
     * 
     * @return Command that spits out the coral.
     */
    public Command spit(boolean manualOverride) {
        return new Command() {

            @Override
            public void initialize() {
                // System.out.println("Spit");
            }

            @Override
            public void execute() {
                spitterMotor.set(SPIT_SPEED);
            }

            @Override
            public boolean isFinished() {
                if (manualOverride) {
                    return false;
                }
                return !hasCoral().getAsBoolean();
            }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
            }
        };
    }

    public Command manualSpit() {
        return new Command() {

            @Override
            public void initialize() {
                // System.out.println("Spit");
            }

            @Override
            public void execute() {
                spitterMotor.set(SPIT_SPEED);
            }

            @Override
            public boolean isFinished() {
                return false;
            }

            @Override
            public void end(boolean interrupted) {
                spitterMotor.stopMotor();
            }
        };
    }
}
