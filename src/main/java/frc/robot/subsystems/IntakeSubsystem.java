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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.MotorControllerIDs;

public class IntakeSubsystem extends SubsystemBase {
  private final CANSparkMax intakeMotor;
  private final CANSparkMax pivotMotor;
  private final CANcoder pivotCANcoder;
  private final CANcoderConfiguration cancoder_cfg;
  private final ProfiledPIDController pivotPID;
  private final ArmFeedforward pivotFeedforward;
  // Flag
  private double setAngle = 0;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(MotorControllerIDs.kIntakeMotorID, MotorType.kBrushless);
    pivotMotor = new CANSparkMax(MotorControllerIDs.kIntakePivotMotorID, MotorType.kBrushless);
    // Reset Setting
    intakeMotor.restoreFactoryDefaults();
    pivotMotor.restoreFactoryDefaults();
    // Mode Setting
    intakeMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setIdleMode(IdleMode.kBrake);
    // Inverted
    intakeMotor.setInverted(false);
    pivotMotor.setInverted(false);
    // Encoder
    pivotCANcoder = new CANcoder(IntakeConstants.kIntakeCANcoderID);
    cancoder_cfg = new CANcoderConfiguration();
    cancoder_cfg.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    cancoder_cfg.MagnetSensor.MagnetOffset = IntakeConstants.kEncoderOffset;
    cancoder_cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    pivotCANcoder.getConfigurator().apply(cancoder_cfg);
    // PID Controller
    pivotPID = new ProfiledPIDController(
      IntakeConstants.kp, 
      IntakeConstants.ki, 
      IntakeConstants.kd,
      new TrapezoidProfile.Constraints(1, 2));  // Rev/s、 Rev/s^2
    // Feedforward
    pivotFeedforward = new ArmFeedforward(
      IntakeConstants.kS, 
      IntakeConstants.kG,
      IntakeConstants.kV);
  }

  public void Intake(){
    intakeMotor.setVoltage(6);
    setAngle = IntakeConstants.kIntakeDownAngle;
  }

  public void Eject(){
    intakeMotor.setVoltage(-6);
    setAngle = IntakeConstants.kIntakeUpAngle;
  }

  public double getAngle(){
    return pivotCANcoder.getAbsolutePosition().getValueAsDouble()*180;
  }

  public void setPivotMotor(double val){
    pivotMotor.set(val);
  }

  public void setIntakeMotor(double val){
    intakeMotor.set(val);
  }

  public void setAngle(double targetAngle){
    setAngle = targetAngle;
  }

  public void stopMotors(){
    intakeMotor.set(0);
    pivotMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double pidOutput = pivotPID.calculate(getAngle(), setAngle);
    pidOutput = Math.abs(pidOutput)>0.5 ? 0.5 : pidOutput;
    pivotMotor.set(pidOutput);
    if(Constants.globalDebug){
      SmartDashboard.putNumber("IntakePivot", getAngle());
    }
  }
}
