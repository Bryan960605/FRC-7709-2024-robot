// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
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
  private final CANSparkMax pivotMotor;
  // NEO encoder
  private final RelativeEncoder shooterEncoder;
  // Pivot CAN Encoder
  private final CANcoder pivotEncoder;
  private final CANcoderConfiguration cancoder_cfg;
  // Note Sensor
  private final DigitalInput IRsensor;

  public ShooterSubsystem() {
    /* IR Sensor */
    IRsensor = new DigitalInput(DigitalInputPin.kIRsensorDIO);
    /* Motor Controller Initialize */
    shooterMotor = new CANSparkMax(MotorControllerIDs.kShooterMotorID, MotorType.kBrushless); 
    feedMotor = new CANSparkMax(MotorControllerIDs.kFeedMotorID, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(MotorControllerIDs.kShooterPivotMotorID, MotorType.kBrushless);
    // Reset Setting
    feedMotor.restoreFactoryDefaults();
    shooterMotor.restoreFactoryDefaults();
    pivotMotor.restoreFactoryDefaults();
    // Mode Setting
    feedMotor.setIdleMode(IdleMode.kBrake);
    shooterMotor.setIdleMode(IdleMode.kCoast);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    // Inverted
    feedMotor.setInverted(false);
    shooterMotor.setInverted(false);
    pivotMotor.setInverted(false);
    // // Burn Flash    
    // feedMotor.burnFlash();
    // shooterMotor.burnFlash();
    // pivotMotor.burnFlash();
    // Encoder
    shooterEncoder = shooterMotor.getEncoder();
    pivotEncoder = new CANcoder(ShooterConstants.kPivotCANcoderID);
    // CANcoder Configuration
    cancoder_cfg = new CANcoderConfiguration();
    cancoder_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoder_cfg.MagnetSensor.MagnetOffset = ShooterConstants.kPivotEncoderOffset;
    cancoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotEncoder.getConfigurator().apply(cancoder_cfg);
    resetEncoder();
  }

  public double getPivotAngle(){
    return pivotEncoder.getAbsolutePosition().getValue()*180;
  }

  public double getPivotVelocity(){
    return pivotEncoder.getVelocity().getValue(); //RPS
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
