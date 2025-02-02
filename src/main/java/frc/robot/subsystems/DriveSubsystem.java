package frc.robot.subsystems;

import frc.hawklibraries.drivetrains.swerve.SwerveDrivetrain;
import frc.hawklibraries.drivetrains.swerve.SwerveDrivetrainConfig;
import frc.hawklibraries.drivetrains.swerve.SwerveModuleConfig;
import frc.hawklibraries.drivetrains.swerve.SwerveModuleConfig.SwerveModuleConfigBuilder;
import frc.hawklibraries.vendorRewrites.wpilib.ChassisSpeeds;

public class DriveSubsystem {

    public SwerveDrivetrain drivetrain;
    private SwerveDrivetrainConfig config;

    private SwerveModuleConfigBuilder<?,?> baseModuleConfig() {

        return SwerveModuleConfig.builder()
            .driveGearRatio(0)
            .maxSpeed(0)
            .wheelDiameter(0)
            .turningGearRatio(0)
            .turningOutputMax(0)
            .turningOutputMin(0);

    }

    public DriveSubsystem() {

        config = SwerveDrivetrainConfig.builder()
            .robotWidth(0)
            .robotLength(0)
            .maxAccel(0)
            .maxSpeed(0)
            .maxTurnRate(0)
            .moduleDistance(0)
            .frontLeftConfig(
                baseModuleConfig()
                .driveID(0)
                .turningID(0)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .frontRightConfig(
                baseModuleConfig()
                .driveID(0)
                .turningID(0)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .backLeftConfig(
                baseModuleConfig()
                .driveID(0)
                .turningID(0)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .backRightConfig(
                baseModuleConfig()
                .driveID(0)
                .turningID(0)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .build();

        drivetrain = new SwerveDrivetrain(
            config,
            () -> {
                return new ChassisSpeeds();
            }
        );
    }
    
}
