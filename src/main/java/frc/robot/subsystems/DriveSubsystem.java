package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

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
            .driveGearRatio(5.9)
            .maxSpeed(MetersPerSecond.of(5.24))
            .wheelDiameter(Inch.of(4))
            .turningGearRatio(12.8)
            .turningOutputMax(-1)
            .turningOutputMin(1);

    }

    public DriveSubsystem() {

        config = SwerveDrivetrainConfig.builder()
            .robotWidth(Inch.of(27))
            .robotLength(Inch.of(27))
            .maxAccel(MetersPerSecondPerSecond.of(1.0))
            .maxSpeed(MetersPerSecond.of(5.24))
            .maxTurnRate(RadiansPerSecond.of(Math.PI))
            .moduleDistance(Inch.of(26))
            .frontLeftConfig(
                baseModuleConfig()
                .driveID(1)
                .turningID(2)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .frontRightConfig(
                baseModuleConfig()
                .driveID(3)
                .turningID(4)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .backLeftConfig(
                baseModuleConfig()
                .driveID(5)
                .turningID(6)
                .invertDriveMotor(false)
                .encoderOffset(null)
                .build()
            )
            .backRightConfig(
                baseModuleConfig()
                .driveID(7)
                .turningID(8)
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
