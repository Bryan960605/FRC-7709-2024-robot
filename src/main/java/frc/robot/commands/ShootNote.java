// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Aiming.DriveAimingTarget;
import frc.robot.commands.Aiming.ShooterAimingTarget;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNote extends SequentialCommandGroup {
  /** Creates a new ShootNote. */
  public ShootNote(SwerveSubsystem m_SwerveSubsystem, VisionSubsystem m_VisionSubsystem, ShooterSubsystem m_ShooterSubsystem) {
    /* Aiming Target Command Group */
    ParallelCommandGroup AimingCommandGroup = new ParallelCommandGroup(
      new DriveAimingTarget(m_SwerveSubsystem, m_VisionSubsystem),
      new ShooterAimingTarget(m_ShooterSubsystem)
    );
    addCommands(
      AimingCommandGroup, // Aiming Target
      new ShooterShoot(m_ShooterSubsystem, m_VisionSubsystem.getTargetType()) // Shoot Note
    );
  }
}
