// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SpitterSubsystem;

public class RobotContainer {

  CommandXboxController driverController;
  ButtonBoard buttonBoard;
  ElevatorSubsystem elevatorSubsystem;
  SpitterSubsystem spitterSubsystem;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    buttonBoard = new ButtonBoard(1);
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

    buttonBoard.b_1().onTrue(
      elevatorSubsystem.goToLevel(ReefHeight.L1)
        .andThen(spitterSubsystem.spit())
      // L1
    );

    buttonBoard.b_2().onTrue(
      elevatorSubsystem.goToLevel(ReefHeight.L2)
        .andThen(spitterSubsystem.spit())
      // L2
    );

    buttonBoard.b_3().onTrue(
      elevatorSubsystem.goToLevel(ReefHeight.L4)
        .andThen(spitterSubsystem.spit())
      // L3
    );

    buttonBoard.b_4().onTrue(
      elevatorSubsystem.goToLevel(ReefHeight.L4)
        .andThen(spitterSubsystem.spit())
      // L4
    );

    buttonBoard.b_5().onTrue(
      Commands.none()
    );

    buttonBoard.b_6().onTrue(
      Commands.none()
    );

    buttonBoard.b_7().onTrue(
      Commands.none()
    );

    buttonBoard.b_8().onTrue(
      Commands.none()
    );

    buttonBoard.b_9().onTrue(
      Commands.none()
    );

    buttonBoard.b_10().onTrue(
      Commands.none()
    );

    buttonBoard.b_11().onTrue(
      Commands.none()
    );

    buttonBoard.b_12().onTrue(
      Commands.none()
    );

    SmartDashboard.putData(elevatorSubsystem);

  }
}