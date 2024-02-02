// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPin;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.MotorControllerIDs;

public class ShooterSubsystem extends SubsystemBase {
  // Motor Controller
  private final CANSparkMax shooterMotorLeft;
  private final CANSparkMax shooterMotorRight;
  private final CANSparkMax feedMotor;
  // NEO encoder
  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  // Note Sensor
  private final DigitalInput IRsensor;
  // PID controller
  private final PIDController shooterPidController;

  private double out=0;

  public ShooterSubsystem() {
    /* IR Sensor */
    IRsensor = new DigitalInput(DigitalInputPin.kIRsensorDIO);
    /* Motor Controller Initialize */
    shooterMotorLeft = new CANSparkMax(MotorControllerIDs.kShooterMotorLeftID, MotorType.kBrushless); 
    shooterMotorRight = new CANSparkMax(MotorControllerIDs.kShooterMotorRightID, MotorType.kBrushless); 
    feedMotor = new CANSparkMax(MotorControllerIDs.kIntakeMotorID, MotorType.kBrushless);
    // Reset Setting
    feedMotor.restoreFactoryDefaults();
    shooterMotorLeft.restoreFactoryDefaults();
    shooterMotorRight.restoreFactoryDefaults();
    // Mode Setting
    feedMotor.setIdleMode(IdleMode.kBrake);
    shooterMotorLeft.setIdleMode(IdleMode.kCoast);
    shooterMotorRight.setIdleMode(IdleMode.kCoast);
    // Inverted
    feedMotor.setInverted(false);
    shooterMotorLeft.setInverted(false);
    shooterMotorRight.setInverted(false);
    // // Burn Flash    
    // feedMotor.burnFlash();
    // shooterMotorLeft.burnFlash();
    // shooterMotorRight.burnFlash();
    // Encoder
    leftMotorEncoder = shooterMotorLeft.getEncoder();
    rightMotorEncoder = shooterMotorRight.getEncoder();
    resetEncoder();
    // PID Controller
    shooterPidController = new PIDController(
      IntakeShooterConstants.kp, 
      IntakeShooterConstants.ki,
      IntakeShooterConstants.kd);
  }

  public void FeedNote(){
    feedMotor.setVoltage(2.5);
  }

  public void ShootSpeaker(){
    // Calculate PID Output
    double PidOutput = shooterPidController.calculate(getEncoderSpeed(), IntakeShooterConstants.kSpeakerRPM);
    // Enable Shooter
    shooterMotorLeft.setVoltage(IntakeShooterConstants.kSpeakerBaseVolt + PidOutput);
    // Feeding Note into Shooter
    if(Math.abs(shooterPidController.getPositionError()) < 20) IntakeFeed();
  }

  public void IntakeFeed(){
    // feedMotor.set(0.2);
  }

  public void setShooterMotor(double value){
    out = value;
    // shooterMotorLeft.set(value);
  }
  
  public void setIntakeMotor(double value){
    // feedMotor.set(value);
  }
  
  // Reset EncoderS
  public void resetEncoder(){
    leftMotorEncoder.setPosition(0);
    // rightMotorEncoder.setPosition(0);
  }
  // Return Motor Speed
  public double getEncoderSpeed(){
    return leftMotorEncoder.getVelocity();
  }
  // Return Sensor State
  public boolean getNoteState(){
    return !IRsensor.get();
  }

  public void stopShooter(){
    // shooterMotorLeft.set(0);
    out=0;
    // shooterMotorRight.set(0);
  }

  public void stopIntake(){
    // feedMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftMotorSpeed", getEncoderSpeed());
  }
}
