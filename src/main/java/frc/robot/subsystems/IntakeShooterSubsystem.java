// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPin;
import frc.robot.Constants.MotorControllerIDs;

public class IntakeShooterSubsystem extends SubsystemBase {
  // Motor Controller
  private final CANSparkMax shooterMotorL;
  private final CANSparkMax shooterMotorR;
  private final CANSparkMax intakeMotor;
  // NEO encoder
  private final RelativeEncoder leftMotorEncoder;
  private final RelativeEncoder rightMotorEncoder;
  // Note Sensor
  private final DigitalInput IRsensor;

  public IntakeShooterSubsystem() {
    /* IR Sensor */
    IRsensor = new DigitalInput(DigitalInputPin.kIRsensorDIO);
    /* Motor Controller Initialize */
    shooterMotorL = new CANSparkMax(MotorControllerIDs.kShooterMotorLeftID, MotorType.kBrushless); // Leader
    shooterMotorR = new CANSparkMax(MotorControllerIDs.kShooterMotorRightID, MotorType.kBrushless); // Follower
    intakeMotor = new CANSparkMax(MotorControllerIDs.kIntakeMotorID, MotorType.kBrushless);
    // Reset Setting
    intakeMotor.restoreFactoryDefaults();
    shooterMotorL.restoreFactoryDefaults();
    shooterMotorR.restoreFactoryDefaults();
    // Mode Setting
    intakeMotor.setIdleMode(IdleMode.kBrake);
    shooterMotorL.setIdleMode(IdleMode.kBrake);
    shooterMotorR.setIdleMode(IdleMode.kBrake);
    // Inverted
    intakeMotor.setInverted(false);
    shooterMotorL.setInverted(false);
    shooterMotorR.setInverted(false);
    // Burn Flash    
    intakeMotor.burnFlash();
    shooterMotorL.burnFlash();
    shooterMotorR.burnFlash();
    // Right motor will follow left motor
    shooterMotorR.follow(shooterMotorL);
    // Encoder
    leftMotorEncoder = shooterMotorL.getEncoder();
    rightMotorEncoder = shooterMotorR.getEncoder();
  }

  public void Intake(){
    intakeMotor.set(0.5);
  }

  public void Shoot(){
    shooterMotorL.set(0.5);
    shooterMotorR.set(0.5);
    if(shooterMotorL.getEncoder().getVelocity() > 2900){
      intakeMotor.set(0.5);
    }
    else{
      intakeMotor.set(0);
    }
  }

  public void setShooterSpeed(double value){
    shooterMotorL.set(value);
  }

  public void stopShooter(){
    shooterMotorL.set(0);
    shooterMotorR.set(0);
  }

  public void setIntakeSpeed(double value){
    intakeMotor.set(value);
  }

  public void stopIntake(){
    intakeMotor.set(0);
  }

  public void resetEncoder(){
    leftMotorEncoder.setPosition(0);
    rightMotorEncoder.setPosition(0);
  }

  // Return Left Motor Speed
  public double getEncoderSpeed(){
    return leftMotorEncoder.getVelocity();
  }
  // Return IR Sensor State
  public boolean getNoteState(){
    return !IRsensor.get();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
