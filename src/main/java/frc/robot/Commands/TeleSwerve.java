// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleSwerve extends Command {
  /** Creates a new TeleSwerve. */
  private Drivetrain swerveBase;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotSup;
  private Boolean robotCentricSup;
  private double slowModeSpeed = 0.25;

  public TeleSwerve(Drivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotSup, Boolean robotCentricSup) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveBase = drivetrain;
    addRequirements(drivetrain);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotSup = rotSup;
    this.robotCentricSup = robotCentricSup;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), 0.1);
    double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), 0.1);
    double rotationVal = MathUtil.applyDeadband(rotSup.getAsDouble(), 0.1);

    swerveBase.drive(
      strafeVal * slowModeSpeed,
      translationVal * slowModeSpeed,
       rotationVal * slowModeSpeed * 4, //was times 4
       !robotCentricSup,
       true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
