// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DigitalInputPin;
import frc.robot.Constants.MotorControllerIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  // Motor Controller
  private final CANSparkMax shooterMotor;
  private final CANSparkMax feedMotor;
  // NEO encoder
  private final RelativeEncoder shooterEncoder;
  // Note Sensor
  private final DigitalInput IRsensor;

  public ShooterSubsystem() {
    /* IR Sensor */
    IRsensor = new DigitalInput(DigitalInputPin.kIRsensorDIO);
    /* Motor Controller Initialize */
    shooterMotor = new CANSparkMax(MotorControllerIDs.kShooterMotorID, MotorType.kBrushless); 
    feedMotor = new CANSparkMax(MotorControllerIDs.kFeedMotorID, MotorType.kBrushless);
    // Reset Setting
    feedMotor.restoreFactoryDefaults();
    shooterMotor.restoreFactoryDefaults();
    // Mode Setting
    feedMotor.setIdleMode(IdleMode.kBrake);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    // Inverted
    feedMotor.setInverted(false);
    shooterMotor.setInverted(false);
    // // Burn Flash    
    // feedMotor.burnFlash();
    // shooterMotor.burnFlash();
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();
    resetEncoder();
  }

  public void FeedNote(){
    feedMotor.set(ShooterConstants.kFeedNotePercent);
  }

  public void ejectNote(){
    feedMotor.set(-ShooterConstants.kFeedNotePercent);
  }

  public void setShooter(double value){
    shooterMotor.set(value);
  }  
  
  public void setFeedMotor(double value){
    feedMotor.setVoltage(value);
  }
  
  // Reset Encoders
  public void resetEncoder(){
    shooterEncoder.setPosition(0);
  }
  // Return Motor Speed
  public double getShooterSpeed(){
    return shooterEncoder.getVelocity();  // RPM
  }

  // Return Sensor State
  public boolean getNoteState(){
    return !IRsensor.get();
  }

  public void stopShooter(){
    shooterMotor.set(0);
  }

  public void stopFeedMotor(){
    feedMotor.set(0);
  }

  public void stopMotors(){
    shooterMotor.set(0);
    feedMotor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LeftShooterSpeed", getShooterSpeed());
    SmartDashboard.putBoolean("NoteState", getNoteState());
  }
}
