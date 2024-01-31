// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.ArmCommand;

import static frc.robot.Constants.ApriltagConstants.*;

public class ArmSubsystem extends SubsystemBase {
  //Motor
  private final CANSparkMax armMotor1 = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax armMotor2 = new CANSparkMax(2, MotorType.kBrushless);
  //Eancoder
  private final CANcoder armCancoder = new CANcoder(0);
  private final CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
  private final RelativeEncoder armEncoder = armMotor1.getEncoder();
  //Controller
  private final ArmFeedforward armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  private final PIDController armPID = new PIDController(0, 0, 0);

  private final double armMaxOutput = 0.3;
  private final double absoluteEncoderOffset = 0;
  private final double toAngularVelocity = Math.PI*2/60;

  private double armFeedforwardOutput;
  private double armPIDOutput;
  private double armMoveOutput;

  private double armAngle;
  private double armRadians;
  private double armErrorValue;

  private double armSetpoint;
  public double armAimSetpoint;
  private double distance;
  private double armAngularVelocity;

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
  }

  public void armPIDCalculate(double setpoint){
    armSetpoint = setpoint;
    if(armErrorValue > 3){
      armMotor1.set(armMoveOutput);
    }
    else{
      armMotor1.set(armFeedforwardOutput);
    }
  }

  public void getObjectDistance(double distance){
    this.distance = distance;
  }

  @Override
  public void periodic() {
    armAngle = armCancoder.getPosition().getValueAsDouble();
    armAngularVelocity = armEncoder.getVelocity()*toAngularVelocity;
    armRadians = Math.toRadians(armAngle);
    armAimSetpoint = -90 + Math.toDegrees(Math.atan((distance + limelightToArmDistance)/(speakerHeight - armHeight))) + (180 - armAndEndEffectorAngle);
    armErrorValue = Math.abs(armSetpoint - armAngle);

    armFeedforwardOutput = armFeedforward.calculate(armRadians, armAngularVelocity)/12;
    armPIDOutput = armPID.calculate(armAngle, armAimSetpoint);
    armPIDOutput = Constants.setMaxOutput(armPIDOutput, armMaxOutput);
    armMoveOutput = armPIDOutput + armFeedforwardOutput;

    SmartDashboard.putString("Mode", ArmCommand.mode);
    SmartDashboard.putNumber("armPIDOutput", armPIDOutput);
    SmartDashboard.putNumber("armFeedforwardOutput", armFeedforwardOutput);
    SmartDashboard.putNumber("armMoveOutput", armMoveOutput);
  }
}
