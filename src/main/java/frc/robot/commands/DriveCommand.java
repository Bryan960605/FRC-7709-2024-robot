// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  // Selw rate limiter
  private final SlewRateLimiter xLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter yLimiter = new SlewRateLimiter(4);
  private final SlewRateLimiter zLimiter = new SlewRateLimiter(4);
  // Variable
  private final DoubleSupplier xSpeedFunc, ySpeedFunc, zSpeedFunc;
  private final BooleanSupplier isFieldRelativeFunc;

  public DriveCommand(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeedFunc, DoubleSupplier ySpeedFunc, DoubleSupplier zSpeedFunc, BooleanSupplier isFieldRelativeFunc) {
    this.m_swerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeedFunc;
    this.ySpeedFunc = ySpeedFunc;
    this.zSpeedFunc = zSpeedFunc;
    this.isFieldRelativeFunc = isFieldRelativeFunc;
    addRequirements(m_swerveSubsystem); 
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Get value
    double xSpeed = xSpeedFunc.getAsDouble();
    double ySpeed = ySpeedFunc.getAsDouble();  
    double zSpeed = zSpeedFunc.getAsDouble();
    boolean isFieldRelative = isFieldRelativeFunc.getAsBoolean();
    // Deadband
    xSpeed = Math.abs(xSpeed) < OperatorConstants.kJoystickDeadband ? 0.0 : xSpeed;
    ySpeed = Math.abs(ySpeed) < OperatorConstants.kJoystickDeadband ? 0.0 : ySpeed;
    zSpeed = Math.abs(zSpeed) < OperatorConstants.kJoystickDeadband ? 0.0 : zSpeed;
    // SlewRate
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    zSpeed = zLimiter.calculate(zSpeed);
    // Output to Drivebase
    m_swerveSubsystem.drive(xSpeed, ySpeed, zSpeed, isFieldRelative);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
