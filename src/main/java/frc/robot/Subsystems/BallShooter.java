// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

public class BallShooter extends SubsystemBase {
  /** Creates a new BallShooter. */
  private SparkMax left = new SparkMax(CANIDs.shooterLeft, MotorType.kBrushless);
  private SparkMax right = new SparkMax(CANIDs.shooterRight, MotorType.kBrushless);

  public BallShooter() {}

  public void spin(double speed){
    left.set(speed);
    right.set(speed);
  }

  public void stop(){
    left.set(0);
    right.set(0);
  }

  public Command shoot(DoubleSupplier power){
    return new RunCommand(()->spin(power.getAsDouble()), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
