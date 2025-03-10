// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.inputs.FlightStick;
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
			DriveSubsystem.MAX_VELOCITY,
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

		configDriveControls();
		configManipControls();
		configSimControls();

		if (RobotBase.isSimulation()) {
			configSimControls();	
		}

	}

	private void configDriveControls() {

		SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
			driveSubsystem.getSwerveDrive(),
			() -> -driverController.a_Y(),
			() -> -driverController.a_X()
		).withControllerRotationAxis(
			() -> -driverController.a_Z()
		)
			.deadband(0.05)
			.cubeRotationControllerAxis(true)
			.cubeTranslationControllerAxis(true)
			.scaleTranslation(0.8)
			.scaleRotation(0.8)
			.allianceRelativeControl(true);

		driveSubsystem.setDefaultCommand(
			driveSubsystem.driveFieldOriented(
				driveAngularVelocity
			)
		);

		driverController.b_10().onTrue(
			driveSubsystem.zeroGyroWithAllianceCommand()
		);

		driverController.b_3().onTrue(
			CoralStationPathing.findHumanZones(driveSubsystem)
		);
    
		driverController.b_Trigger()
			.and(buttonBoard.joy_R())
				.onTrue(
					CoralZones.findCoralZone(true, driveSubsystem)
				);

		driverController.b_Trigger()
			.and(buttonBoard.joy_L())
				.onTrue(
					CoralZones.findCoralZone(false, driveSubsystem)
				);

		driverController.b_Hazard().onTrue(
				Commands.runOnce(() -> {
					driveSubsystem.getCurrentCommand().cancel();
					// cancels ALL DRIVING on driver controller
			})
		);

	}

	private void configManipControls() {

		// Coral Controls

		buttonBoard.b_1().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L1)
		);

		buttonBoard.b_2().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L2)
		);

		buttonBoard.b_3().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L3)
		);

		buttonBoard.b_4().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L4)
		// L4 coral
		);

		buttonBoard.b_9().onTrue(
				coralSubsystem.feedCommand()
			);

		// buttonBoard.b_5().onTrue(
		// 	algaeSubsystem.removeUpperAlgae()
		// 	// remove algae from the upper section of the reef
		// );
		
		// buttonBoard.b_5().onFalse(
		// 	algaeSubsystem.throwUpperAlgae()
		// 	// Throws upper algae
		// );

		// buttonBoard.b_6().onTrue(
		// 	algaeSubsystem.removeLowerAlgae()
		// 	// remove algae from the lower section of the reef
		// );

		// buttonBoard.b_6().onFalse(
		// 	algaeSubsystem.intakeAlgae()
		// 	// intakes algae to a hold
		// );

		// buttonBoard.b_7().onTrue(
		// 	algaeSubsystem.readyForCollection()
		// 	// getting ready to collect from ground		
		// );

		// buttonBoard.b_7().onFalse(
		// 	algaeSubsystem.intakeAlgae()
		// 	// intake/stow algae
		// );

		// buttonBoard.b_8().onTrue(
		// 	algaeSubsystem.scoreAlgae()
		// 	//Scores algae
		// );

	
		// buttonBoard.b_10().onTrue(	
		// 		algaeSubsystem.home()
		// );
		// Sets the algae arm to home


		// Climb Controls

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
	}

	private void configSimControls() {
		driverController.b_5().onTrue(
			Commands.runOnce(
				() -> driveSubsystem.resetOdometry(driveSubsystem.getSwerveDrive().getSimulationDriveTrainPose().get())
				// resets the odometry to the simulation pose
			)
		);
	}

	public void initTelemetery() {
		SmartDashboard.putData(coralSubsystem);
		// SmartDashboard.putData(climbSubsystem);
		// SmartDashboard.putData(algaeSubsystem);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}