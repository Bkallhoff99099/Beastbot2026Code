// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/** Add your docs here. */
public final class Constants {
    public static final class SwerveConstants{
        public static final double driveKS = 0.32;
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        public static final double driveKP = 0.12;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKFF = 0.0;

        public static final double turnKP = 0.12;
        public static final double turnKI = 0.0;
        public static final double turnKD = 0.0;


        public static double driveGearRatio = 6.03;
        public static double turnGearRatio = 26.09;
        public static double wheelDiameter = 4.0;
        public static double wheelCircumference = 2 * Math.PI * (wheelDiameter / 2);


        public static final double LRWheelBase = 24.75;
        public static final double FBWheelBase = 19.75;
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(LRWheelBase / 2.0, FBWheelBase / 2.0), // left front
            new Translation2d(LRWheelBase / 2.0, -FBWheelBase / 2.0), // left back
            new Translation2d(-LRWheelBase / 2.0, FBWheelBase / 2.0), // right front
            new Translation2d(-LRWheelBase / 2.0, -FBWheelBase / 2.0) // right back
        );

        public static final int frontLeftDriveID = 1;
        public static final int backLeftDriveID = 3;
        public static final int backRightDriveID = 5;
        public static final int frontRightDriveID = 7;

        public static final int frontLeftTurnID = 2;
        public static final int backLeftTurnID = 4;
        public static final int backRightTurnID = 6;
        public static final int frontRightTurnID = 8;

        public static final int frontLeftCancoderID = 9;
        public static final int backLeftCancoderID = 10;
        public static final int backRightCancoderID = 11;
        public static final int frontRightCancoderID = 12;

    }

    public static class PIDConstants{
        public static final double intakeInSetpoint = Math.toRadians(90);
        public static final double intakeOutSetpoint = Math.toRadians(0);
    }

    public static class CANIDs{
        public static final int intakeLeft = 13;
        public static final int intakeRight = 14;
        public static final int intakeExtender = 15;
        public static final int indexerLeft = 16;
        public static final int indexerRight = 17;
        public static final int towerLeft = 18;
        public static final int towerRight = 19;
        public static final int shooterLeft = 20;
        public static final int shooterRight = 21;
        public static final int climbLeft = 22;
        public static final int climbRight = 23;
    }

}
