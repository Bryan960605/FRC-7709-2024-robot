// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldObject;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterShoot extends Command {
  private final ShooterSubsystem m_ShooterSubsystem;
  private final FieldObject m_target;
  // Variable
  private double setVoltage = 0;

  public ShooterShoot(ShooterSubsystem shooterSubsystem, FieldObject Target) {
    /* Subsystem */
    this.m_ShooterSubsystem = shooterSubsystem;
    this.m_target = Target;
    // Choose Setpoint
    switch (m_target) {
      case SPEAKER:
        setVoltage = ShooterConstants.kSpeakerVal;
        break;
      case AMP:
        setVoltage = ShooterConstants.kAmpVal;
        break;
      default:
        setVoltage = 0.0;
        break;
    }
    /* Add requirement */
    addRequirements(m_ShooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Soft Start
    m_ShooterSubsystem.setShooter(0.1);
    // Stop FeedMotor
    m_ShooterSubsystem.stopFeedMotor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Shooter Spin up
    m_ShooterSubsystem.setShooter(setVoltage);
    // Feed note when the speed is enough 
    if(m_ShooterSubsystem.getShooterSpeed()>=ShooterConstants.kSpeakerRPM){
      m_ShooterSubsystem.FeedNote();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ShooterSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Retuen true to cancel this Command when the note was sent out
    return m_ShooterSubsystem.getNoteState()==false;
  }
}