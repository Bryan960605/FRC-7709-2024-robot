// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorControllerIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterPivotSubsystem extends SubsystemBase {
  private final CANSparkMax pivotMotor;
  // Pivot CAN Encoder
  private final CANcoder pivotEncoder;
  private final CANcoderConfiguration cancoder_cfg;
  private final PIDController m_PidController;
  private double TargetAngle;

  /** Creates a new ShooterPivotSubsystem. */
  public ShooterPivotSubsystem() {
    pivotMotor = new CANSparkMax(MotorControllerIDs.kShooterPivotMotorID, MotorType.kBrushless);
    pivotMotor.restoreFactoryDefaults();
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    pivotEncoder = new CANcoder(ShooterConstants.kPivotCANcoderID);
    // CANcoder Configuration
    cancoder_cfg = new CANcoderConfiguration();
    cancoder_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoder_cfg.MagnetSensor.MagnetOffset = ShooterConstants.kPivotEncoderOffset;
    cancoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoder.getConfigurator().apply(cancoder_cfg);
    m_PidController = new PIDController(
      ShooterConstants.kp, 
      ShooterConstants.ki, 
      ShooterConstants.kd);
    TargetAngle = 0;
  }

  public double getPivotAngle(){
    return pivotEncoder.getAbsolutePosition().getValue();
  }

  public void setPivotMotor(double val){
    pivotMotor.set(val);
  }

  public void setAngle(double val){
    TargetAngle = val;
  }

  @Override
  public void periodic() {
      // Calculate  
      double output = m_PidController.calculate(getPivotAngle(), TargetAngle);
      // Deadband
      output = Math.abs(output)<0.05 ? 0 : output;
      // Move Pivot
      pivotMotor.set(output);
      SmartDashboard.putNumber("vX_Output", output);
  }
}
