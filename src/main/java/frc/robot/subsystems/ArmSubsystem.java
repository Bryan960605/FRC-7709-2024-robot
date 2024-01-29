// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
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

import static frc.robot.Constants.ApriltagConstants.*;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */
  private final CANSparkMax armMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax armMotor2 = new CANSparkMax(2, MotorType.kBrushless);

  
  private final CANcoder armCANcoder = new CANcoder(0);
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();

  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController armPID = new PIDController(0, 0, 0);

  private final double armMaxOutput = 0.3;
  private final double absoluteEncoderOffset = 0;


  private double armFeedforwardOutput;
  private double armPIDOutput;
  private double armMoveOutput;

  private double armPosition;
  private double distance;
  private double armSetpoing = 0.0;
  public double armAimSetpoint;

  public ArmSubsystem() {
    armMotor2.follow(armMotor1);
    armMotor1.restoreFactoryDefaults();
    armMotor2.restoreFactoryDefaults();

    armMotor1.setIdleMode(IdleMode.kBrake);
    armMotor2.setIdleMode(IdleMode.kBrake);
    
    armMotor1.setInverted(false);
    armMotor2.setInverted(false);

    armMotor1.burnFlash();
    armMotor2.burnFlash();

    cancoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    armCANcoder.getConfigurator().apply(cancoderConfig);
  }
  
  public double armPIDCalculate(double setpoint){
    return Constants.setMaxOutput(armPID.calculate(armPosition, setpoint), armMaxOutput);
  }

  public void setArmAngle(double value){
    armSetpoing = value;
  }

  @Override
  public void periodic() {
    armFeedforwardOutput = armFeedforward.calculate(armMoveOutput, armFeedforwardOutput);
    armPosition = armCANcoder.getPosition().getValueAsDouble();
    armAimSetpoint = -90 + Math.toDegrees(Math.atan((distance + limelightToArmDistance)/(speakerHeight - armHeight)));

    armPIDOutput = armPIDCalculate(armAimSetpoint);
  }
}
