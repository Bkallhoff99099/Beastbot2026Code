// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Commands.TeleSwerve;
import frc.robot.Subsystems.Drivetrain;

public class RobotContainer {
private final XboxController driveController = new XboxController(0);

private final int translationAxis = XboxController.Axis.kLeftY.value;
private final int strafeAxis = XboxController.Axis.kLeftX.value;
private final int rotationAxis = XboxController.Axis.kRightX.value;

private final Drivetrain drivetrain = new Drivetrain();

  public RobotContainer() {
    configureBindings();
    drivetrain.setDefaultCommand(
      new TeleSwerve(drivetrain, 
      ()-> -driveController.getRawAxis(translationAxis),
      ()-> -driveController.getRawAxis(strafeAxis) , 
      ()-> -driveController.getRawAxis(rotationAxis),
      true)
    );
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
