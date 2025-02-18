// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SpitterSubsystem;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class RobotContainer {

	CommandXboxController driverController;
	ButtonBoard buttonBoard;
	ElevatorSubsystem elevatorSubsystem;
	SpitterSubsystem spitterSubsystem;
	DriveSubsystem driveSubsystem;

	public RobotContainer() {

		driverController = new CommandXboxController(0);
		buttonBoard = new ButtonBoard(1);
		elevatorSubsystem = new ElevatorSubsystem();
		spitterSubsystem = new SpitterSubsystem();
		driveSubsystem = new DriveSubsystem(
			"swerve",
			MetersPerSecond.of(3.0),
			new Pose2d(),
			TelemetryVerbosity.HIGH
		);

		configureBindings();
	}

	private void configureBindings() {

		driveSubsystem.setDefaultCommand(
			driveSubsystem.driveCommand(
				driverController::getLeftX,
				driverController::getLeftY,
				driverController::getRightX
			)
		);

		driverController.a().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L1)
			.andThen(spitterSubsystem.spit())
		);

		driverController.b().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L2)
			.andThen(spitterSubsystem.spit())
		);

		driverController.x().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L3)
			.andThen(spitterSubsystem.spit())
		);

		driverController.y().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L4)
			.andThen(spitterSubsystem.spit())
		);

		buttonBoard.b_1().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L1)
			.andThen(spitterSubsystem.spit())
		);

		buttonBoard.b_2().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L2)
			.andThen(spitterSubsystem.spit())
		);

		buttonBoard.b_3().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L3)
			.andThen(spitterSubsystem.spit())
		);

		buttonBoard.b_4().onTrue(
			elevatorSubsystem.goToLevel(ReefHeight.L4)
			.andThen(spitterSubsystem.spit())
		);

		driverController.a()
		.and(buttonBoard.joy_L())
			.onTrue(
				driveSubsystem.findCoralZone(true)
			);

		driverController.a()
		.and(buttonBoard.joy_R())
			.onTrue(
				driveSubsystem.findCoralZone(false)
			);
		
		SmartDashboard.putData(elevatorSubsystem);

	}
}