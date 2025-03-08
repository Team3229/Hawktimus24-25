// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.Timer; // for the backup climb controls
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.inputs.FlightStick;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisualizerSubsystem;
import frc.robot.subsystems.algae.AlgaeSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import frc.robot.utilities.CoralStationPathing;
import frc.robot.utilities.CoralZones;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class RobotContainer {

	FlightStick driverController;
	ButtonBoard buttonBoard;
	CoralSubsystem coralSubsystem;
	DriveSubsystem driveSubsystem;
	// ClimbSubsystem climbSubsystem;
	// AlgaeSubsystem algaeSubsystem;

	VisualizerSubsystem visualizerSubsystem;

	private SendableChooser<Command> autoChooser;

	public RobotContainer() {

		driverController = new FlightStick(0);
		buttonBoard = new ButtonBoard(1);
		// climbSubsystem = new ClimbSubsystem();
		coralSubsystem = new CoralSubsystem();
		// algaeSubsystem = new AlgaeSubsystem();
		driveSubsystem = new DriveSubsystem(
				"swerve",
				MetersPerSecond.of(5.0),
				new Pose2d(2, 4, new Rotation2d()),
				TelemetryVerbosity.HIGH,
				() -> {
					return VisionSubsystem.getMT2Pose(driveSubsystem.getHeading(), driveSubsystem.getRobotVelocity().omegaRadiansPerSecond);
				});

		// visualizerSubsystem = new VisualizerSubsystem(
		// 	() -> coralSubsystem.getElevatorPose().in(Meters),
		// 	() -> coralSubsystem.getFeederAngle().in(Degrees),
		// 	() -> climbSubsystem.getCurrentAngle().in(Degrees),
		// 	() -> algaeSubsystem.getPosition().in(Degrees)
		// );

		configureBindings();
		initTelemetery();
	}

	private void configureBindings() {

		DriverStation.silenceJoystickConnectionWarning(true);

		SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
				driveSubsystem.getSwerveDrive(),
				() -> -driverController.a_Y(),
				() -> -driverController.a_X())
				.withControllerRotationAxis(() -> -driverController.a_Z())
				.deadband(0.1)
				.cubeRotationControllerAxis(true)
				.cubeTranslationControllerAxis(true)
				.scaleTranslation(0.8)
				.scaleRotation(0.6)
				.allianceRelativeControl(true);

		driveSubsystem.setDefaultCommand(
				driveSubsystem.driveFieldOriented(
						driveAngularVelocity));

		driverController.b_10().onTrue(
			Commands.runOnce(
				driveSubsystem::zeroGyroWithAlliance
			)
		);

		buttonBoard.b_1().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L1)
		// L1 coral
		);

		buttonBoard.b_2().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L2)
		// L2 coral
		);

		buttonBoard.b_3().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L3)
		// L3 coral
		);

		buttonBoard.b_4().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L4)
		// L4 coral
		);

		// buttonBoard.b_5().onTrue(
		// 	algaeSubsystem.removeUpperAlgae()
		// 	// remove algae from the upper section of the reef
		// );

		// buttonBoard.b_6().onTrue(
		// 	algaeSubsystem.removeLowerAlgae()
		// 	// remove algae from the lower section of the reef
		// );

		// buttonBoard.b_7().onTrue(
		// 	algaeSubsystem.intakeAlgae()
		// 	// intake/stow algae
		// );

		buttonBoard.b_9().onTrue(
			coralSubsystem.feedCommand()
		);

		// buttonBoard.b_9().onTrue(
		// 	Commands.runOnce(() -> {
		// 		coralSubsystem.spit().schedule();
		// 		// manual eject
		// 	})
		// );

		// buttonBoard.b_10().onTrue(
		// 	Commands.runOnce(() -> {
		// 		driveSubsystem.getCurrentCommand().cancel();
		// 		// cancels ONLY DRIVING on buttonboard
		// 	})
		// );

		// 		//TESTING AND POTENTIAL COMP CLIMB CONTROLS// WORKS IN SIMULATION
		// buttonBoard.joy_U()
		// .and(driverController.b_9()).onTrue(
		// 	Commands.runOnce(() -> {
		// 		climbSubsystem.engageClimb();
		// 		// climb up
		// 	})
		// );

		// buttonBoard.joy_D()
		// .and(driverController.b_9()).onTrue(
		// 	Commands.runOnce(() -> {
		// 		climbSubsystem.disengageClimb();
		// 		// climb down
		// 	})
		// );

				//ALTERNATIVE CLIMB CONTROLS // CANNOT TEST UNLESS DURING PRACTICE (or real) MATCH
		// buttonBoard.joy_U().whileTrue(
		// 	Commands.runOnce(() -> {
		// 		if(Timer.getMatchTime() < 130) {
		// 			climbSubsystem.engageClimb();
		// 		}
		// 		System.out.println("Climb Up");
		// 	})
		// );

		// buttonBoard.joy_D().whileTrue(
		// 	Commands.runOnce(() -> {
		// 		if(Timer.getMatchTime() < 130) {
		// 			climbSubsystem.engageClimb();
		// 		}
		// 		System.out.println("Climb Down");
		// 	})
		// );

		driverController.b_3().onTrue(
			Commands.runOnce(() -> {
				CoralStationPathing.findHumanZones(driveSubsystem).schedule();
				// drive to the coral human player zones
			})
		);
    
		driverController.b_Trigger()
		.and(buttonBoard.joy_R())
			.onTrue(
				Commands.runOnce(() -> {
					CoralZones.findCoralZone(true, driveSubsystem).schedule();
				}, driveSubsystem
				// drive to the right coral zones
			));

		driverController.b_Trigger()
		.and(buttonBoard.joy_L())
			.onTrue(
				Commands.runOnce(() -> {
					CoralZones.findCoralZone(false, driveSubsystem).schedule();
				}, driveSubsystem
				// drive to the left coral zones
			));

		driverController.b_Hazard().onTrue(
				Commands.runOnce(() -> {
					driveSubsystem.getCurrentCommand().cancel();
					// cancels ALL DRIVING on driver controller
			})
		);
		if (RobotBase.isSimulation()) {
			driverController.b_5().onTrue(
				Commands.runOnce(
					() -> driveSubsystem.resetOdometry(driveSubsystem.getSwerveDrive().getSimulationDriveTrainPose().get()),
					driveSubsystem
					// resets the odometry to the simulation pose
				)
			);
		}
	}

	public void initTelemetery() {
		SmartDashboard.putData(coralSubsystem);
		// SmartDashboard.putData(climbSubsystem);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}