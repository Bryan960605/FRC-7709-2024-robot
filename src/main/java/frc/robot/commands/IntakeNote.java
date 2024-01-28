// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeShooterSubsystem;

public class IntakeNote extends Command {
  private IntakeShooterSubsystem m_IntakeShooterSubsystem;
  private ArmSubsystem m_ArmSubsystem;

  public IntakeNote(IntakeShooterSubsystem intakeShooterSubsystem, ArmSubsystem armSubsystem) {
    this.m_IntakeShooterSubsystem = intakeShooterSubsystem;
    this.m_ArmSubsystem = armSubsystem;
    addRequirements(m_IntakeShooterSubsystem, m_ArmSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntakeShooterSubsystem.setIntakeSpeed(-0.5);
    m_ArmSubsystem.setArmAngle(0);
  }

  @Override
  public void execute() {
    if(m_IntakeShooterSubsystem.getNoteState()==true) m_IntakeShooterSubsystem.stopIntake();
  }

  @Override
  public void end(boolean interrupted) {
    m_IntakeShooterSubsystem.stopIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
