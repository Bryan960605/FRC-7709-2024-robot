// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimingAMP extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  public AimingAMP(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_VisionSubsystem = visionSubsystem;
    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_SwerveSubsystem.stopModules();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double[] move;
    // Get target pid output
    move = m_VisionSubsystem.AimingAMP();
    // Move drivebase
    // m_SwerveSubsystem.stopModules();
    m_SwerveSubsystem.drive(move[0], move[1], move[2], false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
