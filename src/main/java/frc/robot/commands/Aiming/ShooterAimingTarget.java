// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aiming;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ApriltagIDs;
import frc.robot.Constants.FieldObject;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterPivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class ShooterAimingTarget extends Command {
  private final ShooterPivotSubsystem m_ShooterPivotSubsystem;
  private final VisionSubsystem m_VisionSubsystem;
  private double output=0;

  /** Creates a new ShooterAimingTarget. */
  public ShooterAimingTarget(ShooterPivotSubsystem shooterPivotSubsystem, VisionSubsystem visionSubsystem) {
    this.m_ShooterPivotSubsystem = shooterPivotSubsystem;
    this.m_VisionSubsystem = visionSubsystem;
    addRequirements(m_ShooterPivotSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_VisionSubsystem.isTargetGet()){
      System.out.println("Shooter saw a Target!");
      // Knowing what i'm aiming at.
      int targetID = m_VisionSubsystem.getTargetID();
      FieldObject TargetType = ApriltagIDs.getIDType(targetID);
      // Make sure it's Our alliance
      if(m_VisionSubsystem.isOurAlliance(targetID)){
        System.out.println("Aiming");
        // Get Setpoint
        double TargetAngle = m_VisionSubsystem.calculateShooterAngle();
      }
    }else{
      m_ShooterPivotSubsystem.setAngle(ShooterConstants.kSPEAKER_Angle);
      System.out.println("I'm seeing Nothing!, In front of Speaker");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return output==0;
  }
}
