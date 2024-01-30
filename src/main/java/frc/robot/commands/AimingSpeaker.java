// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AimingSpeaker extends Command {
  private final SwerveSubsystem m_SwerveSubsystem;
  private final VisionSubsystem m_VisionSubsystem;


  public AimingSpeaker(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_VisionSubsystem = visionSubsystem;
    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  @Override
  public void initialize() {
    m_SwerveSubsystem.stopModules();
  }

  @Override
  public void execute(){
    double[] move = null;
    
    // Get target pid output
    move = m_VisionSubsystem.AimingSPEAKER();
    // Move drivebase
    m_SwerveSubsystem.stopModules();
    // m_SwerveSubsystem.drive(move[0], move[1], move[2], false);
  }

  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
