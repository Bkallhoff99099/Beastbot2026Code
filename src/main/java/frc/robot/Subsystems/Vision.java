// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.studica.frc.AHRS;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightHelpers;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private String name;
  private AHRS gyro;
  private SwerveDrivePoseEstimator poseEstimator;
  private boolean doRejectUpdate;


  public Vision(String name, AHRS gyro, SwerveDrivePoseEstimator poseEstimator,  double[] config) {
    this.name = name;
    this.gyro = gyro;
    this.poseEstimator = poseEstimator;

    LimelightHelpers.setCameraPose_RobotSpace(name, config[0], config[1], config[2], config[3], config[4], config[5]);
  }

  public double aimWithVision(){
    double kP = 0.0017;
    double targetingAngularVelocity = LimelightHelpers.getTX(name) * kP;
    targetingAngularVelocity *= 4; // multiplied by max angular velocity
    targetingAngularVelocity *= -1; //inverted
    return targetingAngularVelocity;
  }

  public double rangeWithVision(double range){
    double kP = 0.1;
    double targetingForwardSpeed = (LimelightHelpers.getTY(name) - range) * kP;
    targetingForwardSpeed *= 4.5; // multiplied by max drivetrain velocity
    return targetingForwardSpeed;
  }

  public Pose2d getPose(){
    return LimelightHelpers.getBotPose2d_wpiBlue(name);
  }

  public void setValidTargets(int[] IDs){
    LimelightHelpers.SetFiducialIDFiltersOverride(name, IDs);
  }

  @Override
  public void periodic() {
     doRejectUpdate = false;
    LimelightHelpers.SetRobotOrientation(name,
     poseEstimator.getEstimatedPosition().getRotation().getDegrees(),
      0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate megaTag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    if(Math.abs(gyro.getRate()) > 360){
      doRejectUpdate = true;
    }
    if(megaTag2.tagCount == 0){
      doRejectUpdate = true;
    }
    if(!doRejectUpdate){
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7, 9999999));
      poseEstimator.addVisionMeasurement(megaTag2.pose, megaTag2.timestampSeconds);
    }
    // This method will be called once per scheduler run
  }
}
