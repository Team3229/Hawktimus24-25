package frc.robot.subsystems.drive;

import org.ironmaple.utils.FieldMirroringUtils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class PIDSwerve extends Command {

    private final DriveSubsystem s_Swerve;
    private final Pose2d targetPose;
    private final PIDController xPID, yPID;
    private final PIDController rotationPID = new PIDController(0, 0, 0);

    private double translationKP = 0.0;
    private double positionKS = 0.0;
    private double positionIZone = 0.0;
    private double positionTolerance = 0.0;

    private double rotationIZone = 0.0;
    private double rotationKS = 0.0;
    private double rotationTolerance = 0.0;

    public PIDSwerve(DriveSubsystem s_Swerve, Pose2d targetPose, boolean flipIfRed) {
        super();

        if (flipIfRed) {
            targetPose = FieldMirroringUtils.toCurrentAlliancePose(targetPose);
        }

        this.s_Swerve = s_Swerve;
        this.targetPose = targetPose;
        addRequirements(s_Swerve);

        xPID = new PIDController(translationKP, 0, 0);
        yPID = new PIDController(translationKP, 0, 0);

        xPID.setIZone(positionIZone); // Only use Integral term within this range
        xPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        xPID.setSetpoint(Units.metersToInches(targetPose.getX()));
        xPID.setTolerance(positionTolerance);

        yPID.setIZone(positionIZone); // Only use Integral term within this range
        yPID.setIntegratorRange(-positionKS * 2, positionKS * 2);
        yPID.setSetpoint(Units.metersToInches(targetPose.getY())); // TODO Set derivative, too
        yPID.setTolerance(positionTolerance);

        rotationPID.enableContinuousInput(-180.0, 180.0);
        rotationPID.setIZone(rotationIZone); // Only use Integral term within this range
        rotationPID.setIntegratorRange(rotationKS * 2, rotationKS * 2);
        rotationPID.setSetpoint(targetPose.getRotation().getDegrees());
        rotationPID.setTolerance(rotationTolerance); // TODO Set derivative, too
    }

    @Override
    public void initialize() {
        super.initialize();

        xPID.reset();
        yPID.reset();
        rotationPID.reset();

    }

    @Override
    public void execute() {
        Pose2d pose = s_Swerve.getPose();
        Translation2d position = pose.getTranslation();
        Rotation2d rotation = pose.getRotation();

        /* TODO Consider a potential need to rotate most of the way first, then translate */

        double xCorrection = xPID.calculate(Units.metersToInches(position.getX()));
        double xFeedForward = positionKS * Math.signum(xCorrection);
        double xVal = MathUtil.clamp(xCorrection + xFeedForward, -1.0, 1.0);

        double yCorrection = yPID.calculate(Units.metersToInches(position.getY()));
        double yFeedForward = positionKS * Math.signum(yCorrection);
        double yVal = MathUtil.clamp(yCorrection + yFeedForward, -1.0, 1.0);

        double correction = rotationPID.calculate(rotation.getDegrees());
        double feedForward = rotationKS * Math.signum(correction);
        double rotationVal = MathUtil.clamp(correction + feedForward, -1.0, 1.0);
        
        /* Drive */
        s_Swerve.driveFieldOriented(
            () -> new ChassisSpeeds(xVal, yVal, rotationVal)
        );
    }

    @Override
    public boolean isFinished() {
        return xPID.atSetpoint() && yPID.atSetpoint() && rotationPID.atSetpoint();
    }

    @Override
    public String getName() {
        return "PID Swerve to " + targetPose;
    }
}