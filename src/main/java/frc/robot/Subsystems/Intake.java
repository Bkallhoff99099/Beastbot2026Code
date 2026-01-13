// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANIDs;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax left = new SparkMax(CANIDs.intakeLeft, MotorType.kBrushless);
  private SparkMax right = new SparkMax(CANIDs.intakeRight, MotorType.kBrushless);
  private SparkMax extender = new SparkMax(CANIDs.intakeExtender, MotorType.kBrushless);
  private PIDController pidController = new PIDController(0.12, 0.0, 0.0);
  private AbsoluteEncoder absEncoder;
  private double feedForward = 0.0;
  
  public Intake() {
    absEncoder = extender.getAbsoluteEncoder();
  }

  public void setPower(double speed){
    left.set(speed);
    right.set(speed);
  }

  public void setSetPoint(double setPoint){
    pidController.setSetpoint(setPoint);
  }

  public void runPID(){
      double angle = absEncoder.getPosition();
      angle = angle > 190 ? angle - 360 : angle;
      double output = pidController.calculate(angle);
      output += feedForward * Math.cos(absEncoder.getPosition());
      extender.set(output);
  }

  public Command spin(DoubleSupplier speed){
    return new RunCommand(()->setPower(speed.getAsDouble()), this);
  }

  @Override
  public void periodic() {
    runPID();
  }
}
