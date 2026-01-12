// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.concurrent.locks.ReentrantLock;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] swerveMods;
  public AHRS gyro;

  private final ReentrantLock swerveModLock = new ReentrantLock();


  public Drivetrain() {
    gyro = new AHRS();
    gyro.setYaw(0);

    swerveMods = new SwerveModule[]{
      new SwerveModule(1, 2, 1, null),
      new SwerveModule(3, 4, 2, null),
      new SwerveModule(5, 6, 3, null),
      new SwerveModule(7, 8, 4, null)
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
    swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), heading);
  }

  public Command zeroHeading(){
    return Commands.runOnce(
      ()->{
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
      }, this);
  }

  public Rotation2d getGyroYaw(){
    return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    swerveOdometry.update(getGyroYaw(), getModulePositions());

  
  }
}
