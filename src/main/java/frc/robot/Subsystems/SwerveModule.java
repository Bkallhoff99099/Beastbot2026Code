// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utils.CTREConfigs;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private TalonFX driveMotor;
  private TalonFX turnMotor;
  private CANcoder angleEncoder;
  private SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(
    SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);
  private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
  private final VelocityVoltage driveVelocity = new VelocityVoltage(0);
  private final PositionVoltage anglePosition = new PositionVoltage(0);
  private Rotation2d angleOffset;

  public SwerveModule(int driveCAN, int turnCAN, int canCoderID, Rotation2d angleOffset) {

    angleEncoder = new CANcoder(canCoderID);
    angleEncoder.getConfigurator().apply(CTREConfigs.getCANcoderConfig());

    turnMotor = new TalonFX(turnCAN);
    turnMotor.getConfigurator().apply(CTREConfigs.getTurnConfig());

    driveMotor = new TalonFX(driveCAN);
    driveMotor.getConfigurator().apply(CTREConfigs.getDriveConfig());
    driveMotor.getConfigurator().setPosition(0.0);

    this.angleOffset = angleOffset;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
    desiredState.optimize(getState().angle);
    PositionVoltage request = anglePosition.withPosition(desiredState.angle.getRotations());
    turnMotor.setControl(request);
    setSpeed(desiredState, isOpenLoop);
  }

  public void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
    if(isOpenLoop){
      driveDutyCycle.Output = desiredState.speedMetersPerSecond / 4.5; // divided by max meters per second
      driveMotor.setControl(driveDutyCycle);
    }else{
      driveVelocity.Velocity = desiredState.speedMetersPerSecond / SwerveConstants.wheelCircumference;
      driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
      driveMotor.setControl(driveVelocity);
    }
  }

  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValueAsDouble());
  }

  public void resetToAbsolute(){
    Timer.delay(1);
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    turnMotor.setPosition(absolutePosition);
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(
      (driveMotor.getVelocity().getValueAsDouble() * SwerveConstants.wheelCircumference),
      Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble())
    );
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      (driveMotor.getPosition().getValueAsDouble() * SwerveConstants.wheelCircumference),
      Rotation2d.fromRotations(turnMotor.getPosition().getValueAsDouble())
    );
  }

  public void runMotor(int ID){
    if(ID == driveMotor.getDeviceID()){
      driveMotor.set(0.3);
    }
    if(ID == turnMotor.getDeviceID()){
      turnMotor.set(0.3);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

