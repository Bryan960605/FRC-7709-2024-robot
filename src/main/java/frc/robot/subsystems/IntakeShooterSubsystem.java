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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPin;
import frc.robot.Constants.IntakeShooterConstants;
import frc.robot.Constants.MotorControllerIDs;

public class IntakeShooterSubsystem extends SubsystemBase {
  // Motor Controller
  private final CANSparkMax shooterMotorLead;
  private final CANSparkMax shooterMotorFollow;
  private final CANSparkMax intakeMotor;
  // NEO encoder
  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  // Note Sensor
  private final DigitalInput IRsensor;
  // PID controller
  private final PIDController shooterPidController;

  public IntakeShooterSubsystem() {
    /* IR Sensor */
    IRsensor = new DigitalInput(DigitalInputPin.kIRsensorDIO);
    /* Motor Controller Initialize */
    shooterMotorLead = new CANSparkMax(MotorControllerIDs.kShooterMotorLeftID, MotorType.kBrushless); // Leader
    shooterMotorFollow = new CANSparkMax(MotorControllerIDs.kShooterMotorRightID, MotorType.kBrushless); // Follower
    intakeMotor = new CANSparkMax(MotorControllerIDs.kIntakeMotorID, MotorType.kBrushless);
    // Reset Setting
    intakeMotor.restoreFactoryDefaults();
    shooterMotorLead.restoreFactoryDefaults();
    shooterMotorFollow.restoreFactoryDefaults();
    // Mode Setting
    intakeMotor.setIdleMode(IdleMode.kBrake);
    shooterMotorLead.setIdleMode(IdleMode.kBrake);
    shooterMotorFollow.setIdleMode(IdleMode.kBrake);
    // Inverted
    intakeMotor.setInverted(false);
    shooterMotorLead.setInverted(false);
    shooterMotorFollow.setInverted(false);
    // Burn Flash    
    intakeMotor.burnFlash();
    shooterMotorLead.burnFlash();
    shooterMotorFollow.burnFlash();
    // Right motor will follow left motor
    shooterMotorFollow.follow(shooterMotorLead);
    // Encoder
    leftMotorEncoder = shooterMotorLead.getEncoder();
    rightMotorEncoder = shooterMotorFollow.getEncoder();
    resetEncoder();
    // PID Controller
    shooterPidController = new PIDController(
      IntakeShooterConstants.kp, 
      IntakeShooterConstants.ki,
      IntakeShooterConstants.kd);
  }

  public void IntakeNote(){
    intakeMotor.set(0.5);
  }

  public void ShootAMP(){
    // Calculate PID Output
    double PidOutput = shooterPidController.calculate(getEncoderSpeed(), IntakeShooterConstants.kAmpRPM);
    // Enable Shooter
    shooterMotorLead.setVoltage(IntakeShooterConstants.kAmpBaseVolt + PidOutput);
    // Feeding Note into Shooter
    if(Math.abs(shooterPidController.getPositionError()) < 20) IntakeFeed();
  }

  public void ShootSpeaker(){
    // Calculate PID Output
    double PidOutput = shooterPidController.calculate(getEncoderSpeed(), IntakeShooterConstants.kSpeakerRPM);
    // Enable Shooter
    shooterMotorLead.setVoltage(IntakeShooterConstants.kSpeakerBaseVolt + PidOutput);
    // Feeding Note into Shooter
    if(Math.abs(shooterPidController.getPositionError()) < 20) IntakeFeed();
  }

  public void IntakeFeed(){
    intakeMotor.set(0.2);
  }

  public void setShooterMotor(double value){
    shooterMotorLead.set(value);
  }
  
  public void setIntakeMotor(double value){
    intakeMotor.set(value);
  }
  
  // Reset EncoderS
  public void resetEncoder(){
    leftMotorEncoder.setPosition(0);
    rightMotorEncoder.setPosition(0);
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
    shooterMotorLead.set(0);
    shooterMotorFollow.set(0);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
