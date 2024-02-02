// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import javax.swing.text.StyleContext.SmallAttributeSet;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterShoot extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final ProfiledPIDController shooterPID;
  public ShooterShoot(ShooterSubsystem intakeShooterSubsystem) {
    /* Subsystem */
    this.m_ShooterSubsystem = intakeShooterSubsystem;
    /* PID Controller */
    shooterPID = new ProfiledPIDController(
      0.0, 
      0, 
      0, 
      new TrapezoidProfile.Constraints(100, 20));
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ShooterSubsystem.stopIntake();
    // m_ShooterSubsystem.setShooterMotor(0.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* Get wheel velocity and setpoint */
    double wheelMeasurement = m_ShooterSubsystem.getEncoderSpeed();
    double wheelsetpoint = 100;
    double baseOutput = 0.5;
    /* PID Calculation */
    // double PIDoutput = shooterPID.calculate(wheelMeasurement, wheelsetpoint);
    double PIDoutput = 0;
    /* Output to Motor */
    m_ShooterSubsystem.setShooterMotor(baseOutput + PIDoutput);
    SmartDashboard.putNumber("OUTPUT", PIDoutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
