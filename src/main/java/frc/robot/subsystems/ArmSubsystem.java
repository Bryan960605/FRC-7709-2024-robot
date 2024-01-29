// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorControllerIDs;

public class ArmSubsystem extends SubsystemBase {
  /* Motor controllers */
  private final CANSparkMax armMotorLead = new CANSparkMax(MotorControllerIDs.kArmMotorLeftID, MotorType.kBrushless);
  private final CANSparkMax armMotorFollow = new CANSparkMax(MotorControllerIDs.kArmMotorRightID, MotorType.kBrushless);
  /* Encoder */
  private final CANcoder encoder = new CANcoder(0);
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
  /* Feedforward */
  private final ArmFeedforward armFeedforward = new ArmFeedforward(
    ArmConstants.kS,
    ArmConstants.kG,
    ArmConstants.kV);
  /* PID Controller */
  private final PIDController armPID = new PIDController(
    ArmConstants.kp, 
    ArmConstants.ki, 
    ArmConstants.kd);
  /* Arm Setpoint */
  private double setPoint = 0.0; // Units:Degree

  public ArmSubsystem() {
    /* Motor Controller Setting */
    armMotorFollow.follow(armMotorLead);
    armMotorLead.restoreFactoryDefaults();
    armMotorFollow.restoreFactoryDefaults();
    // Set idle mode
    armMotorLead.setIdleMode(IdleMode.kBrake);
    armMotorFollow.setIdleMode(IdleMode.kBrake);
    // Set inverted
    armMotorLead.setInverted(false);
    armMotorFollow.setInverted(false);
    // Burn flash
    armMotorLead.burnFlash();
    armMotorFollow.burnFlash();
    /* CANcoder configuration */
    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    cancoderConfig.MagnetSensor.MagnetOffset = ArmConstants.KArmEncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(cancoderConfig);
  }

  public double getAngleDegree(){
    return encoder.getAbsolutePosition().getValueAsDouble()*360;
  }
  
  public double getAngleRadian(){
    return encoder.getAbsolutePosition().getValueAsDouble()*Math.PI;
  }

  public double getVelocityRadPerSec(){
    return encoder.getVelocity().getValueAsDouble()*2*Math.PI;
  }
  
  public void IdleInsidePosition(){
    setPoint = ArmConstants.IdleOutsideAngle;
  }

  public void IdleOutsidePosition(){
    setPoint = ArmConstants.IdleOutsideAngle;
  }

  public void AimSPEAKER(){
    // CalculateAngle

    // Setpoint
  }

  public void AimAMP(){
    // CalculateAngle

    // Setpoint
  }

  public void groundIntake(){
    setPoint = ArmConstants.groundIntakeAngle;
  }

  public double PidCalculate(double setpoint){
    return Constants.setMaxOutput(armPID.calculate(getAngleDegree(), setpoint), ArmConstants.kArmMaxOutput);
  }

  public void setAngleDegree(double value){
    setPoint = value;
  }

  public void setMotorVolt(double value){
    armMotorLead.setVoltage(value);
  }

  @Override
  public void periodic() {
    // Feedforward Calculation
    double FeedForward = armFeedforward.calculate(getAngleRadian(), getVelocityRadPerSec());
    // PID Calculation
    double PIDController = PidCalculate(setPoint);
    // Output to Motors
    setMotorVolt(FeedForward + PIDController);
  }
}
