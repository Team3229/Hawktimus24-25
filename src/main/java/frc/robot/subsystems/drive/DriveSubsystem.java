// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inch;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.hawklibraries.utilities.Alliance;
import frc.hawklibraries.utilities.Alliance.AllianceColor;
import frc.robot.constants.ReefHeight;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.coral.ElevatorSubsystem;
import frc.robot.utilities.LimelightHelpers.PoseEstimate;

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
			8.5,
			0.0,
			0.5
		);

	private static final PIDConstants ROTATION_CONSTANTS =
		new PIDConstants(
			6.5,
			0.0,
			0.0
		);

	private static final PIDConstants PP_TRANS = 
		new PIDConstants(
			4.25,
			0.0,
			0.01
		);

	// private static final PIDConstants PP_TRANS = 
	// 	new PIDConstants(
	// 		3.75,
	// 		0.0,
	// 		0.015
	// 	);

	private static final PIDConstants PP_ROT = 
		new PIDConstants(
			3.0,
			0.0,
			0.0
		);

	private static final double TRANSLATION_ERROR_TOLERANCE = 0.2;
	private static final double TRANSLATION_VELOCITY_TOLERANCE = 0.005;
	private static final double ROTATION_ERROR_TOLERANCE = Degrees.of(0.25).in(Radians);
	private static final double ROTATION_VELOCITY_TOLERANCE = 0.01;

    private PIDController xTranslationPID = new PIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD
    );

	private PIDController yTranslationPID = new PIDController(
        TRANSLATION_CONSTANTS.kP,
        TRANSLATION_CONSTANTS.kI,
        TRANSLATION_CONSTANTS.kD
    );

    private PIDController rotationPID = new PIDController(
        ROTATION_CONSTANTS.kP,
        ROTATION_CONSTANTS.kI,
        ROTATION_CONSTANTS.kD
    );

    private CoralZones coralZones = new CoralZones();
	private AlgaeZones algaeZones = new AlgaeZones();
    private CoralStationPathing coralStationPathing = new CoralStationPathing();

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

		xTranslationPID.setTolerance(TRANSLATION_ERROR_TOLERANCE, TRANSLATION_VELOCITY_TOLERANCE);
		yTranslationPID.setTolerance(TRANSLATION_ERROR_TOLERANCE, TRANSLATION_VELOCITY_TOLERANCE);
		rotationPID.setTolerance(ROTATION_ERROR_TOLERANCE, ROTATION_VELOCITY_TOLERANCE);

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

		swerveDrive.setHeadingCorrection(true);
		swerveDrive.setCosineCompensator(RobotBase.isReal());
		swerveDrive.setAngularVelocityCompensation(
				true,
				true,
				0.1);
		swerveDrive.setModuleEncoderAutoSynchronize(
				false,
				1);

		swerveDrive.pushOffsetsToEncoders();

		swerveDrive.setAutoCenteringModules(true);

		if (RobotBase.isSimulation()) {
			swerveDrive.getMapleSimDrive().get().config.bumperLengthX = Inch.of(33.954922);
			swerveDrive.getMapleSimDrive().get().config.bumperWidthY = Inch.of(33.954922);
		}

		setupPathPlanner();

		SmartDashboard.putData("XPID", xTranslationPID);
		SmartDashboard.putData("YPID", yTranslationPID);
		SmartDashboard.putData("RPID", rotationPID);

		PathPlannerLogging.setLogActivePathCallback((poses) -> {
			// Do whatever you want with the poses here
			swerveDrive.field.getObject("Trajectory").setPoses(poses);
        });
	}

	@Override
	public void periodic() {
		PoseEstimate estimate = VisionSubsystem.getMT2Pose(getPose().getRotation(), swerveDrive.getRobotVelocity().omegaRadiansPerSecond);

		if (estimate != null) {
			swerveDrive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds);
		}
	}

	/**
	 * Setup AutoBuilder for PathPlanner.
	 */
	public void setupPathPlanner() {

		NamedCommands.registerCommand("DriveToLeft", driveToReef(true));
		NamedCommands.registerCommand("DriveToRight", driveToReef(false));

		RobotConfig config;

		try {
			config = RobotConfig.fromGUISettings();

			final boolean enableFeedforward = true;

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
					(Pose2d pose) -> {
						Pose2d noRotPose = new Pose2d(pose.getX(), pose.getY(),
							
							(Alliance.getAlliance() == AllianceColor.Blue) ? swerveDrive.getOdometryHeading() : swerveDrive.getOdometryHeading().rotateBy(Rotation2d.fromDegrees(180))
						);
						resetOdometry(noRotPose);
					},
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
		return driveFieldOriented(
            getInputStream(
                () -> restrictToMax(xTranslationPID.calculate(getPose().getX(), pose.get().getX()) / MAX_VELOCITY.in(MetersPerSecond), 0.2),
                () -> restrictToMax(yTranslationPID.calculate(getPose().getY(), pose.get().getY()) / MAX_VELOCITY.in(MetersPerSecond), 0.2),
                () -> restrictToMax(rotationPID.calculate(getPose().getRotation().getRadians(), pose.get().getRotation().getRadians()) / swerveDrive.getMaximumChassisAngularVelocity(), 0.5)
            ).allianceRelativeControl(false)

        ).ignoringDisable(false)
		.until(
			() -> xTranslationPID.atSetpoint() && yTranslationPID.atSetpoint() && rotationPID.atSetpoint()
		);
    }

	private double restrictToMax(double in, double restrict) {
		if (Math.abs(in) > Math.abs(restrict)) {
			return (in < 0) ? -restrict : restrict;
		} else {
			return in;
		}
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
	public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
		return run(() -> {
			swerveDrive.driveFieldOriented(velocity.get());
		}).ignoringDisable(false);
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
		return swerveDrive.getPose();
	}

	/**
	 * Resets the gyro angle to zero and resets odometry to the same position, but
	 * facing toward 0.
	 */
	public void zeroGyro() {
		swerveDrive.zeroGyro();
	}

	/**
	 * This will zero (calibrate) the robot to assume the current position is facing
	 * forward
	 * <p>
	 * If red alliance rotate the robot 180 after the drviebase zero command
	 */
	public void zeroGyroWithAlliance() {

		if (Alliance.getAlliance() == AllianceColor.Red) {
			zeroGyro();
			// Set the pose 180 degrees
			resetOdometry(new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(180)));
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

				Rotation2d mt1 = VisionSubsystem.getMT1Rotation();

				if (mt1 != null) {
					swerveDrive.setGyro(new Rotation3d(mt1));
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
        return driveToPose(() -> {
			return coralZones.findCoralZone(leftSide, getPose());
		})
		.andThen(
			() -> {
				SmartDashboard.putBoolean("Done Lining Up", true);
			}
		);
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