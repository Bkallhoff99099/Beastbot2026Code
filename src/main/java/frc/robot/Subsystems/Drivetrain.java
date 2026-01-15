// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.concurrent.locks.ReentrantLock;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;
  public ADIS16470_IMU gyro;

  private final ReentrantLock swerveModLock = new ReentrantLock();


  public Drivetrain() {
    gyro = new ADIS16470_IMU();
    gyro.reset();

    swerveMods = new SwerveModule[]{
      new SwerveModule(SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftCancoderID, new Rotation2d()),
      new SwerveModule(SwerveConstants.backLeftDriveID, SwerveConstants.backLeftTurnID, SwerveConstants.backLeftCancoderID, new Rotation2d()),
      new SwerveModule(SwerveConstants.frontRightDriveID, SwerveConstants.frontRightTurnID, SwerveConstants.frontRightCancoderID, new Rotation2d()),
      new SwerveModule(SwerveConstants.backRightDriveID, SwerveConstants.backRightTurnID, SwerveConstants.backRightCancoderID, new Rotation2d())
    };

    swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getGyroYaw(), getModulePositions());

  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = 
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
          translation.getX(), 
          translation.getY(),
          rotation,
          getHeading()
          ) 
          : new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation)
          );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.5);

      swerveModLock.lock();
      int i = 0;
      for(SwerveModule mod : swerveMods){
        
        mod.setDesiredState(swerveModuleStates[i], isOpenLoop);
        i++;
      }
      swerveModLock.unlock();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean isOpenLoop){
    SwerveModuleState[] swerveModuleStates = 
      SwerveConstants.swerveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(
          ySpeed,
          xSpeed,
          rot
        )
      );

      SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.5);

      swerveModLock.lock();
      int i = 0;
      for(SwerveModule mod : swerveMods){
        mod.setDesiredState(swerveModuleStates[i], isOpenLoop);

        i++;
      }
      swerveModLock.unlock();
  }

  public void setModuleStates(SwerveModuleState[] desiredStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, 4.5);
    int i = 0;
    for(SwerveModule mod : swerveMods){
     
      mod.setDesiredState(desiredStates[i], false);
       i++;
    }
  }

  public SwerveModuleState[] getModuleStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];

    swerveModLock.lock();
    int i = 0;
    for(SwerveModule mod : swerveMods){
    
      states[i] = mod.getState();
        i++;
    }
    swerveModLock.unlock();
    return states;
  }

  public SwerveModulePosition[] getModulePositions(){
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    swerveModLock.lock();
    int i = 0;
    for(SwerveModule mod : swerveMods){
      
      positions[i] = mod.getPosition();
      i++;
    }
    swerveModLock.unlock();

    return positions;
  }

  public Pose2d getPose(){
    return swerveOdometry.getPoseMeters();
  }

  public void setPose(Pose2d pose){
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
  }

  public Rotation2d getHeading(){
    return getPose().getRotation();
  }

  public void setHeading(Rotation2d heading){
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
  }

  public Command zeroHeading(){
    return Commands.runOnce(
      ()->{
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
      }, this);
  }

  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(gyro.getAngle());
  }

  public Command resetModulesToAbsolute(){
    return Commands.runOnce(
      ()->{
        swerveModLock.lock();
        int i = 0;
        for(SwerveModule mod :swerveMods){
          
          mod.resetToAbsolute();
          i++;
        }
        swerveModLock.unlock();
      }, this);
  }

  private void updateSwerveOdom(){
    swerveOdometry.update(getGyroYaw(), getModulePositions());
  }

  public void setXMode(){
    swerveMods[0].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
    swerveMods[1].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
    swerveMods[2].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)), false);
    swerveMods[3].setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)), false);
  }

 

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateSwerveOdom();

  
  }
}
