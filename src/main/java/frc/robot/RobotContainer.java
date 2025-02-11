// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.constants.ReefHeight;
import frc.robot.inputs.ButtonBoard;
import frc.robot.subsystems.coral.CoralSubsystem;

public class RobotContainer {

  CommandXboxController driverController;
  ButtonBoard buttonBoard;
  CoralSubsystem coralSubsystem;

  public RobotContainer() {
    driverController = new CommandXboxController(0);
    buttonBoard = new ButtonBoard(1);
    coralSubsystem = new CoralSubsystem();
    configureBindings();
    initTelemetery();
  }

  private void configureBindings() {

    driverController.a().onTrue(
        coralSubsystem.elevatorSpit(ReefHeight.L1));

    driverController.b().onTrue(
        coralSubsystem.elevatorSpit(ReefHeight.L2));

    driverController.x().onTrue(
        coralSubsystem.elevatorSpit(ReefHeight.L3));

    driverController.y().onTrue(
        coralSubsystem.elevatorSpit(ReefHeight.L4));

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

  public void initTelemetery(){
      SmartDashboard.putData(coralSubsystem);
    }

}