// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ReefHeight;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SpitterSubsystem;

public class RobotContainer {

  CommandXboxController driverController;
  ElevatorSubsystem elevatorSubsystem;
  SpitterSubsystem spitterSubsystem;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    elevatorSubsystem = new ElevatorSubsystem(driverController::getLeftY);
    spitterSubsystem = new SpitterSubsystem();
    configureBindings();
  }

  private void configureBindings() {

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

    SmartDashboard.putData(elevatorSubsystem);

  }
}