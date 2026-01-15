// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleSwerve;
import frc.robot.Subsystems.BallShooter;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;

public class RobotContainer {
private final XboxController driveController = new XboxController(0);
private final CommandXboxController operatorController = new CommandXboxController(1);

private final int translationAxis = XboxController.Axis.kLeftY.value;
private final int strafeAxis = XboxController.Axis.kLeftX.value;
private final int rotationAxis = XboxController.Axis.kRightX.value;

private final Drivetrain drivetrain = new Drivetrain();
private final BallShooter shooter = new BallShooter();
private final Intake intake = new Intake();



  public RobotContainer() {
    configureBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands(){
     drivetrain.setDefaultCommand(
      new TeleSwerve(drivetrain, 
      ()-> -driveController.getRawAxis(translationAxis),
      ()-> -driveController.getRawAxis(strafeAxis) , 
      ()-> -driveController.getRawAxis(rotationAxis),
      true)
    );

    shooter.setDefaultCommand(
      new RunCommand(()->shooter.spin(0.3), shooter)
    );
  }

  private void configureBindings() {
    new Trigger(()->driveController.getRightTriggerAxis() > 0.3).whileTrue(
      new ParallelCommandGroup(
        shooter.shoot(()->driveController.getRightTriggerAxis()),
        new RunCommand(()->drivetrain.setXMode())
      )
    );

    new Trigger(()->driveController.getLeftTriggerAxis() > 0.1).whileTrue(intake.spin(()->driveController.getLeftTriggerAxis()));

    new Trigger(()->driveController.getAButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[0].runMotor(2), drivetrain));
    new Trigger(()->driveController.getBButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[1].runMotor(4), drivetrain));
    new Trigger(()->driveController.getYButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[2].runMotor(8), drivetrain));
    new Trigger(()->driveController.getXButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[3].runMotor(6), drivetrain));
    new Trigger(()->driveController.getLeftBumperButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[0].runMotor(1), drivetrain));
    new Trigger(()->driveController.getRightBumperButton()).onTrue(new RunCommand(()->drivetrain.swerveMods[1].runMotor(3), drivetrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
