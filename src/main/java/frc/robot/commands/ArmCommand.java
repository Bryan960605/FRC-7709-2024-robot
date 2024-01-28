// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;

public class ArmCommand extends Command {
  /** Creates a new ArmCommand. */
  private final ArmSubsystem armSubsystem;
  public static String mode = "";
  private int button;
  public ArmCommand(ArmSubsystem _armsubsystem) {
    this.armSubsystem = _armsubsystem;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // for(int butt = 1;butt<=10;butt++){
    //   if(RobotContainer.armJoystick.getRawButton(butt)){
    //     button = butt;
    //   }
    // }
    // switch (button) {
    //   case OperatorConstants.floorButton:
    //     armSubsystem.armPIDCalculate(ArmConstants.armTaking);
    //     mode = "floor";
    //     break;
    //   case OperatorConstants.shootButton:
    //     armSubsystem.shoot();
    //     armSubsystem.armPIDCalculate(armSubsystem.armAimSetpoint);
    //     mode = "shoot";
    //     break;
    //   case OperatorConstants.primetiveButton:
    //     armSubsystem.armPIDCalculate(ArmConstants.armOrigin);
    //     mode = "primetive";
    //     break;
    //   case OperatorConstants.takeButton:
    //     armSubsystem.take();
    //     armSubsystem.armPIDCalculate(ArmConstants.armTaking);
    //     mode = "take";
    //     break;
    //   default:
    //     armSubsystem.armPIDCalculate(armSubsystem.armAimSetpoint);
    //     mode = "aim";
    //     break;
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
