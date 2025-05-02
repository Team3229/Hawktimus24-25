// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.utilities.LimelightHelpers;

import java.io.File;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

/**
 * The SwerveDrivetrain class represents a swerve drive subsystem for a robot
 * using YAGSL.It provides methods to control the robot's movement, including
 * driving to specific poses, following trajectories, and handling various
 * driving commands.This class integrates with PathPlanner for autonomous path
 * planning and execution. It also supports telemetry, odometry, and various
 * drive modes such as field-relative and robot-relative control.
 */
public class DriveSubsystem extends SubsystemBase {

	public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(5.0);

	private static final PIDConstants TRANSLATION_CONSTANTS =
		new PIDConstants(
			5.5,
			0.2,
			0.1
		);

	private static final PIDConstants ROTATION_CONSTANTS =
		new PIDConstants(
			8.0,
			0.0,
			0.0
		);

	private static final PIDConstants PP_TRANS = 
		new PIDConstants(
			5.5,
			0.2,
			0.1
		);

	private static final PIDConstants PP_ROT = 
		new PIDConstants(
			3.0,
			0.0,
			0.0
		);

	private static final Distance TRANS_ERR_TOL = Meters.of(0.25);
	private static final LinearVelocity TRANS_VEL_TOL = MetersPerSecond.of(0.1);
	private static final Angle ROT_ERR_TOL = Degrees.of(0.1);
	private static final AngularVelocity ROT_VEL_TOL = DegreesPerSecond.of(0.1);

	private static final LinearVelocity TRANS_MAX_VEL = MetersPerSecond.of(1);
	private static final LinearAcceleration TRANS_MAX_ACCEL = MetersPerSecondPerSecond.of(2);

	private static final AngularVelocity ROT_MAX_VEL = DegreesPerSecond.of(720);
	private static final AngularAcceleration ROT_MAX_ACCEL = DegreesPerSecondPerSecond.of(720);

    private ProfiledPIDController xTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );

	private ProfiledPIDController yTranslationPID = new ProfiledPIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD,
		new Constraints(TRANS_MAX_VEL.in(MetersPerSecond), TRANS_MAX_ACCEL.in(MetersPerSecondPerSecond))
    );

    private ProfiledPIDController rotationPID = new ProfiledPIDController(
        ROTATION_CONSTANTS.kP,
        ROTATION_CONSTANTS.kI,
        ROTATION_CONSTANTS.kD,
		new Constraints(ROT_MAX_VEL.in(RadiansPerSecond), ROT_MAX_ACCEL.in(RadiansPerSecondPerSecond))
    );

    private CoralZones coralZones = new CoralZones();
	private AlgaeZones algaeZones = new AlgaeZones();
    private CoralStationPathing coralStationPathing = new CoralStationPathing();

	public static final int[] CORAL_STATION_APRILTAG_IDS = {1, 2, 12, 13};
    public static final int[] REEF_APRILTAG_IDS = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};

	/**
	 * Swerve drive object.
	 */
	private SwerveDrive swerveDrive;

	/**
	 * Initialize {@link SwerveDrive}
	 *
	 * @param parser             The parser to create the swerve drive.
	 * @param maxChassisVelocity The maximum chassis velocity of the robot.
	 * @param initialPose        The initial pose of the robot.
	 * @param verbosity          The verbosity level for telemetry.
	 */
	public DriveSubsystem(
			String path,
			TelemetryVerbosity verbosity) {

		super();

		SmartDashboard.putBoolean("Done Lining Up", false);

		rotationPID.enableContinuousInput(0, 2 * Math.PI);

		xTranslationPID.setTolerance(TRANS_ERR_TOL.in(Meters), TRANS_VEL_TOL.in(MetersPerSecond));
		yTranslationPID.setTolerance(TRANS_ERR_TOL.in(Meters), TRANS_VEL_TOL.in(MetersPerSecond));
		rotationPID.setTolerance(ROT_ERR_TOL.in(Radians), ROT_VEL_TOL.in(RadiansPerSecond));

		xTranslationPID.setIZone(0.5);
		yTranslationPID.setIZone(0.5);
		rotationPID.setIZone(Degrees.of(20).in(Radians));

		// Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
		// objects being created.
		SwerveDriveTelemetry.verbosity = verbosity;

		try {
			swerveDrive = new SwerveParser(
					new File(Filesystem.getDeployDirectory(), path))
					.createSwerveDrive(
							MAX_VELOCITY.in(MetersPerSecond),
							new Pose2d(2, 4, new Rotation2d()));
		} catch (IOException e) {
			e.printStackTrace();
		}
		
		resetOdometry(new Pose2d(2, 4, swerveDrive.getYaw()));

		if (RobotBase.isSimulation()) {
			swerveDrive.field.setRobotPose(new Pose2d(2, 4, new Rotation2d()));
		}

		swerveDrive.setHeadingCorrection(false);
		swerveDrive.setCosineCompensator(RobotBase.isReal());
		swerveDrive.setAngularVelocityCompensation(
				true,
				true,
				0.1);
		swerveDrive.setModuleEncoderAutoSynchronize(
				false,
				1);

		swerveDrive.useExternalFeedbackSensor();

		swerveDrive.setAutoCenteringModules(false);

		if (RobotBase.isSimulation()) {
			swerveDrive.getMapleSimDrive().get().config.bumperLengthX = Inch.of(33.954922);
			swerveDrive.getMapleSimDrive().get().config.bumperWidthY = Inch.of(33.954922);
		}

		setupPathPlanner();

		SmartDashboard.putData("XPID", xTranslationPID);
		SmartDashboard.putData("YPID", yTranslationPID);
		SmartDashboard.putData("RPID", rotationPID);

		// for (ReefPositions reef : ReefPositions.values()) {
		// 	swerveDrive.field.getObject(reef.name()).setPose(reef.getPosition());
		// }

		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			swerveDrive.field.getObject("Trajectory").setPoses(poses);
        });
	}

	@Override
	public void periodic() {
		
		updateOdometry();

		// PoseEstimate estimate = VisionSubsystem.getMT2Pose(getPose().getRotation(), swerveDrive.getRobotVelocity().omegaRadiansPerSecond, "coral");

		// if (estimate != null) {
		// 	swerveDrive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
		// }

		SmartDashboard.putNumber("X-Pos-Err", xTranslationPID.getPositionError());
		SmartDashboard.putNumber("Y-Pos-Err", yTranslationPID.getPositionError());
		SmartDashboard.putNumber("Z-Pos-Err", rotationPID.getPositionError());

		SmartDashboard.putNumber("X-Vel-Err", xTranslationPID.getVelocityError());
		SmartDashboard.putNumber("Y-Vel-Err", yTranslationPID.getVelocityError());
		SmartDashboard.putNumber("Z-Vel-Err", rotationPID.getVelocityError());

	}

	/**
   * Updates the drivetrain odometry object to the robot's current position on the
   * field.
   * 
   * @return The new updated pose of the robot.
   */
  public void updateOdometry() {

    for (String side : new String[] {"coral", "algae"}) {

      LimelightHelpers.SetRobotOrientation("limelight-" + side, getIMUYaw().getDegrees(), getIMUYawRate().in(DegreesPerSecond), 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-" + side);

      if (estimate != null && estimate.tagCount > 0) {
        
        double tagID = LimelightHelpers.getFiducialID("limelight-" + side);

        for (int desiredID : liningUpToReef() ? REEF_APRILTAG_IDS : CORAL_STATION_APRILTAG_IDS) {

          if (tagID == desiredID) {

            Translation3d aprilTagPosition = LimelightHelpers.getTargetPose3d_RobotSpace("limelight-" + side).getTranslation();

            if (Math.hypot(aprilTagPosition.getX(), aprilTagPosition.getZ()) <= 3.5) {
                
              swerveDrive.addVisionMeasurement(new Pose2d(estimate.pose.getX(), estimate.pose.getY(), getIMUYaw()), estimate.timestampSeconds);
                
            }

            break;

          }

        }

      }

    }

  }

  public Rotation2d getIMUYaw() {
	return getIMU().getRotation2d();
  }

  public AngularVelocity getIMUYawRate() {
	return getIMU().getAngularVelocityZWorld().getValue();
  }

  public void setIMUYaw(Rotation2d yaw) {
	getIMU().setYaw(yaw.getMeasure());
	swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), yaw));
  }

  public Pigeon2 getIMU() {
	return ((Pigeon2) swerveDrive.getGyro().getIMU());
  }

  public boolean liningUpToReef() {
	return true;
	// return getCurrentCommand() != null && getCurrentCommand().getName() == "Reef Lineup";
  }


	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {

		NamedCommands.registerCommand("DriveToLeft",
			driveToReef(true).withTimeout(1)
		);
		NamedCommands.registerCommand("DriveToRight",
			driveToReef(false).withTimeout(1)
		);

		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = false;

			// Configure AutoBuilder last
			AutoBuilder.configure(
					() -> {
						if (RobotBase.isSimulation()) {
							return swerveDrive.field.getRobotPose();
						} else {
							return swerveDrive.getPose();
						}
					},
					// Robot pose supplier
					this::resetOdometry,
					// Method to reset odometry (will be called if your auto has a starting pose)
					() -> swerveDrive.getRobotVelocity(),
					// ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
					(speedsRobotRelative, moduleFeedForwards) -> {
						if (enableFeedforward) {
							swerveDrive.drive(
									speedsRobotRelative,
									swerveDrive.kinematics.toSwerveModuleStates(speedsRobotRelative),
									moduleFeedForwards.linearForces());
						} else {
							swerveDrive.setChassisSpeeds(speedsRobotRelative);
						}
					},
					new PPHolonomicDriveController(
						PP_TRANS,
						PP_ROT
					),
					config,
					() -> {
						return Alliance.getAlliance() == AllianceColor.Red;
					},
					this);

		} catch (Exception e) {
			// Handle exception as needed
			e.printStackTrace();
		}

		// Preload PathPlanner Path finding
		// IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
		PathfindingCommand.warmupCommand().schedule();
	}

	/**
	 * Get the path follower with events.
	 *
	 * @param pathName PathPlanner path name.
	 * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
	 */
	public Command getAutonomousCommand(String pathName) {
		// Create a path following command using AutoBuilder. This will also trigger
		// event markers.
		return new PathPlannerAuto(pathName);
	}

	/**
	 * Use PID control to go to a point on the field.
	 *
	 * @param pose Target {@link Pose2d} to go to.
	 * @return PID command
	 */
	public Command driveToPose(Supplier<Pose2d> pose) {

		return
			runOnce(
				() -> {
					xTranslationPID.reset(getPose().getX(), swerveDrive.getFieldVelocity().vxMetersPerSecond);
					yTranslationPID.reset(getPose().getY(), swerveDrive.getFieldVelocity().vyMetersPerSecond);
					rotationPID.reset(getIMUYaw().getRadians(), swerveDrive.getFieldVelocity().omegaRadiansPerSecond);
				}
			).andThen(
				driveFieldOriented(
					() -> {
						return new ChassisSpeeds(
							xTranslationPID.calculate(getPose().getX(), pose.get().getX()),
							yTranslationPID.calculate(getPose().getY(), pose.get().getY()),
							rotationPID.calculate(getIMUYaw().getRadians(), pose.get().getRotation().getRadians())
						);
					}
				)
			)
			.ignoringDisable(false)
			.until(
				() -> xTranslationPID.atGoal() && yTranslationPID.atGoal() && rotationPID.atGoal()
			);
    }

	/**
	 * Returns a Command that centers the modules of the SwerveDrive subsystem.
	 *
	 * @return a Command that centers the modules of the SwerveDrive subsystem
	 */
	public Command centerModulesCommand() {
		return run(() -> Arrays.asList(swerveDrive.getModules())
				.forEach(it -> it.setAngle(0.0)));
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public Command driveFieldOriented(SwerveInputStream velocity, DoubleSupplier scaleTranslation, DoubleSupplier scaleRotation) {
		return run(() -> {
			swerveDrive.driveFieldOriented(
				velocity
					.scaleTranslation(scaleTranslation.getAsDouble())
					.scaleRotation(scaleRotation.getAsDouble())
					.get()
			);

		}).ignoringDisable(false);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(
				velocity
					.get()
			);
		}).ignoringDisable(false);
	}

	/**
	 * Drive the robot given a chassis field oriented velocity.
	 *
	 * @param velocity Velocity according to the field.
	 */
	public void driveFieldOriented(ChassisSpeeds velocity) {
		
		swerveDrive.driveFieldOriented(velocity);

	}

	/**
	 * Resets odometry to the given pose. Gyro angle and module positions do not
	 * need to be reset when calling this
	 * method. However, if either gyro angle or module position is reset, this must
	 * be called in order for odometry to
	 * keep working.
	 *
	 * @param initialHolonomicPose The pose to set the odometry to
	 */
	public void resetOdometry(Pose2d pose) {
		if (pose == null) {
			swerveDrive.resetOdometry(new Pose2d(2, 4, new Rotation2d()));
			return;
		}
		swerveDrive.resetOdometry(pose);
	}

	/**
	 * Gets the current pose (position and rotation) of the robot, as reported by
	 * odometry.
	 *
	 * @return The robot's pose
	 */	
	public Pose2d getPose() {
		if (RobotBase.isSimulation()) {
			return swerveDrive.field.getRobotPose();
		}
		return new Pose2d(swerveDrive.getPose().getX(), swerveDrive.getPose().getY(), getIMUYaw());
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		getIMU().setYaw(0);
		swerveDrive.resetOdometry(new Pose2d(getPose().getX(), getPose().getY(), new Rotation2d()));
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing
	 * forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drivebase zero command
	 */
	public void zeroGyroWithAlliance() {
		if (Alliance.getAlliance() == AllianceColor.Red) {
			setIMUYaw(new Rotation2d(Math.PI));
		} else {
			zeroGyro();
		}
	}

	public Command zeroGyroWithAllianceCommand() {
		return runOnce(
			this::zeroGyroWithAlliance
		);
	}

	public Command zeroGyroWithLimelight() {

		return runOnce(
			() -> {

				Rotation2d mt1_coral = VisionSubsystem.getMT1Rotation("coral");
				// Rotation2d mt1_algae = VisionSubsystem.getMT1Rotation("algae");

				// if (mt1_coral != null && mt1_algae != null) {
				// 	Rotation2d average = Rotation2d.fromDegrees((mt1_coral.getDegrees() + mt1_algae.getDegrees()) / 2);
				// 	swerveDrive.setGyro(new Rotation3d(average));
				// } else if (mt1_coral != null) {
				// 	swerveDrive.setGyro(new Rotation3d(mt1_coral));
				// } else if (mt1_algae != null) {
				// 	swerveDrive.setGyro(new Rotation3d(mt1_algae));
				// }


				if (mt1_coral != null) {
					if (Alliance.getAlliance() == AllianceColor.Red) {
						setIMUYaw(mt1_coral.rotateBy(new Rotation2d(Math.PI)));
					} else {
						setIMUYaw(mt1_coral);
					}
				}
			}
		);
	}

	public void postTrajectoryToField(List<Pose2d> trajectory) {
		swerveDrive.field.getObject("Trajectory").setPoses(trajectory);
	}

	/**
	 * Lock the swerve drive to prevent it from moving.
	 */
	public void lock() {
		swerveDrive.lockPose();
	}

	public void addVisionReading(Pose2d pose, Time timestamp) {
		swerveDrive.addVisionMeasurement(pose, timestamp.in(Milliseconds));
	}

    public Command driveToReef(boolean leftSide) {
        return 
		runOnce(
			() -> SmartDashboard.putBoolean("Done Lining Up", false)
		).andThen(
		driveToPose(
			() -> coralZones.findCoralZone(leftSide, getPose())
		))
		.andThen(
			() -> {
				SmartDashboard.putBoolean("Done Lining Up", true);
			}
		).withName("Reef Lineup");
    }

	public Command driveToAlgaeZone() {
		return driveToPose(() -> {
			return algaeZones.findAlgaeZone(getPose());
		});
	}

	public Command driveToPlayerStation() {
		return driveToPose(() -> {
			return coralStationPathing.getTargetStation(getPose());
		});
	}

	public SwerveInputStream getInputStream(
		DoubleSupplier x,
		DoubleSupplier y,
		DoubleSupplier rot
	) {
		return new SwerveInputStream(swerveDrive, x, y, rot);
	}

}