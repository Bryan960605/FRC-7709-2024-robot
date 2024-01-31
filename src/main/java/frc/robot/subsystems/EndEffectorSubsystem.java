// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EndEffectorSubsystem extends SubsystemBase {
  /** Creates a new EndeffectorCommand. */
  private final CANSparkMax shooterMotor1 = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax shooterMotor2 = new CANSparkMax(123, MotorType.kBrushless);
  private final CANSparkMax intakeMotor = new CANSparkMax(456, MotorType.kBrushless);

  private final RelativeEncoder shooterEncoder = shooterMotor1.getEncoder();

  private final PIDController shooterPID = new PIDController(0, 0, 0);

  private double shooterPIDOutput;
  private double shooterVelocity;
  public EndEffectorSubsystem() {
    shooterMotor2.follow(shooterMotor1);

    shooterMotor1.restoreFactoryDefaults();
    shooterMotor2.restoreFactoryDefaults();
    intakeMotor.restoreFactoryDefaults();

    shooterMotor1.setInverted(false);
    shooterMotor2.setInverted(false);
    intakeMotor.setInverted(false);

    shooterMotor1.setIdleMode(IdleMode.kCoast);
    shooterMotor2.setIdleMode(IdleMode.kCoast);
    intakeMotor.setIdleMode(IdleMode.kCoast);

    shooterMotor1.burnFlash();
    shooterMotor2.burnFlash();
    intakeMotor.burnFlash();
  }

  public void shoot(){
    shooterMotor1.setVoltage(9.6 + shooterPIDOutput);
    if(shooterVelocity > 4700){
      intakeMotor.set(0.8);
    }
  }

  @Override
  public void periodic() {
    shooterVelocity = shooterEncoder.getVelocity();
    shooterPIDOutput = shooterPID.calculate(shooterVelocity, 4800);
  }
}
