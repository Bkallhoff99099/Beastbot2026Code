// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
    public static TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
    public static CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

    public CTREConfigs(){
        swerveCANcoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        swerveAngleFXConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        swerveAngleFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        swerveAngleFXConfig.Feedback.SensorToMechanismRatio = 26.09; // angle gear ratio
        swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = 30;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 40;
        swerveAngleFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        swerveAngleFXConfig.Slot0.kP = SwerveConstants.turnKP;
        swerveAngleFXConfig.Slot0.kI = SwerveConstants.turnKI;
        swerveAngleFXConfig.Slot0.kD = SwerveConstants.turnKD;

        swerveDriveFXConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        swerveDriveFXConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        swerveDriveFXConfig.Feedback.SensorToMechanismRatio = 6.03; // drive gear ratio
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = 40;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = 60;
        swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = 0.1;
        swerveDriveFXConfig.Slot0.kP = SwerveConstants.driveKP;
        swerveDriveFXConfig.Slot0.kI = SwerveConstants.driveKI;
        swerveDriveFXConfig.Slot0.kD = SwerveConstants.driveKI;
        swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.25;
        swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
        swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.0;
        swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    }
}
