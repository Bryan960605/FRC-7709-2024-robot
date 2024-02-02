// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Aiming;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AimingSetpoint;
import frc.robot.Constants.ApriltagIDs;
import frc.robot.Constants.FieldObject;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class DriveAimingTarget extends Command {
  private SwerveSubsystem m_SwerveSubsystem;
  private VisionSubsystem m_VisionSubsystem;
  /* PID Controller */
  private final PIDController xAimPID;
  private final PIDController yAimPID;
  private final PIDController zAimPID;
  private double vX, vY, vZ;

  public DriveAimingTarget(SwerveSubsystem swerveSubsystem, VisionSubsystem visionSubsystem) {
    this.m_SwerveSubsystem = swerveSubsystem;
    this.m_VisionSubsystem = visionSubsystem;
    xAimPID = new PIDController(0.6, 0, 0);
    yAimPID = new PIDController(0.6, 0, 0);
    zAimPID = new PIDController(0.4, 0, 0);
    addRequirements(m_SwerveSubsystem, m_VisionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vX=0;
    vY=0;
    vZ=0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_VisionSubsystem.isTargetGet()){
      System.out.println("I saw a Target!");
      // Knowing what i'm aiming at
      int targetID = m_VisionSubsystem.getTargetID();
      FieldObject TargetType = ApriltagIDs.getIDType(targetID);
      // Make sure it's Our alliance
      if(m_VisionSubsystem.isOurAlliance(targetID)){
        System.out.println("Aiming");
        // Get Setpoint
        double[] setpoint = AimingSetpoint.setpointSelector(TargetType);
        // Get Target Measurement
        double targetX = m_VisionSubsystem.getTarget3dPose().getX();
        double targetY = m_VisionSubsystem.getTarget3dPose().getY();
        double targetYaw = m_VisionSubsystem.getTargetYaw();
        // Calculate
        vX = xAimPID.calculate(targetX, setpoint[0]);
        vY = yAimPID.calculate(targetY, setpoint[1]);
        vZ = zAimPID.calculate(targetYaw, setpoint[2]);
        // Deadband
        vX = Math.abs(vX)<0.05 ? 0 : vX;
        vY = Math.abs(vY)<0.05 ? 0 : vY;
        vZ = Math.abs(vZ)<0.05 ? 0 : vZ;
        // Move Drivebase
        SmartDashboard.putNumber("vX_Output", vX);
        SmartDashboard.putNumber("vY_Output", vY);
        SmartDashboard.putNumber("vZ_Output", vZ);
        m_SwerveSubsystem.drive(vX, vY, vZ, false);
      }
    }else{
      m_SwerveSubsystem.stopModules();
      System.out.println("I'm seeing Nothing!");
    }
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
