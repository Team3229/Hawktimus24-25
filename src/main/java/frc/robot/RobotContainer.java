// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.inputs.FlightStick;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.VisualizerSubsystem;
import frc.robot.subsystems.coral.CoralSubsystem;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class RobotContainer {

	FlightStick driverController;
	ButtonBoard buttonBoard;
	CoralSubsystem coralSubsystem;
	DriveSubsystem driveSubsystem;
	ClimbSubsystem climbSubsystem;
	AlgaeSubsystem algaeSubsystem;

	VisualizerSubsystem visualizerSubsystem;

	private SendableChooser<Command> autoChooser;

	public RobotContainer() {

		driverController = new FlightStick(0);
		buttonBoard = new ButtonBoard(1);
		climbSubsystem = new ClimbSubsystem();
		coralSubsystem = new CoralSubsystem(driverController.b_3());
		algaeSubsystem = new AlgaeSubsystem();
		driveSubsystem = new DriveSubsystem(
				"swerve",
				MetersPerSecond.of(5.0),
				new Pose2d(2, 4, new Rotation2d()),
				TelemetryVerbosity.HIGH,
				() -> {
					return VisionSubsystem.getMT2Pose(driveSubsystem.getHeading(), 0);
				});

		visualizerSubsystem = new VisualizerSubsystem(
				() -> coralSubsystem.getElevatorPos().in(Meters),
				coralSubsystem::getFeederAngle,
				climbSubsystem::getPosition,
				algaeSubsystem::getPosition);

		configureBindings();
		initTelemetery();
	}

	private void configureBindings() {

		SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
				driveSubsystem.getSwerveDrive(),
				() -> driverController.a_Y() * -1,
				() -> driverController.a_X() * -1)
				.withControllerRotationAxis(() -> -driverController.a_Z())
				.deadband(0.1)
				.scaleTranslation(0.8)
				.allianceRelativeControl(true);

		driveSubsystem.setDefaultCommand(
				driveSubsystem.driveFieldOriented(
						driveAngularVelocity));
		
		driverController.b_Trigger().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L4)
		);

		driverController.b_Hazard().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L3)
		);

		driverController.b_3().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L2)
		);

		driverController.b_4().onTrue(
			coralSubsystem.elevatorSpit(ReefHeight.L1)
		);

		buttonBoard.b_1().onTrue(
				coralSubsystem.elevatorSpit(ReefHeight.L1)
		// L1
		);

		buttonBoard.b_2().onTrue(
				coralSubsystem.elevatorSpit(ReefHeight.L2)
		// L2
		);

		buttonBoard.b_3().onTrue(
				coralSubsystem.elevatorSpit(ReefHeight.L3)
		// L3
		);

		buttonBoard.b_4().onTrue(
				coralSubsystem.elevatorSpit(ReefHeight.L4)
		// L4
		);

		buttonBoard.b_5().onTrue(
				Commands.none());

		buttonBoard.b_6().onTrue(
				Commands.none());

		buttonBoard.b_7().onTrue(
				Commands.none());

		buttonBoard.b_8().onTrue(
				Commands.none());

		buttonBoard.b_9().onTrue(
				Commands.none());

		buttonBoard.b_10().onTrue(
				Commands.none());

		buttonBoard.b_11().onTrue(
				Commands.none());

		buttonBoard.b_12().onTrue(
				Commands.none());
	}

	public void initTelemetery() {
		SmartDashboard.putData(coralSubsystem);

		autoChooser = AutoBuilder.buildAutoChooser();
		SmartDashboard.putData(autoChooser);
	}

	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

}