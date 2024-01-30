// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  //Subsystem
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem();
  private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
  private final VisionSubsystem m_visionSubsystem = new VisionSubsystem();
  // Chooser
  private final SendableChooser<Command> autoChooser;
  //Joystick
  public static final XboxController operatorJoysrick = new XboxController(OperatorConstants.kOperatorJoystickrPort);
  public static final XboxController driverJoystick = new XboxController(OperatorConstants.kDriverJoystickrPort);
  //Button
  public static final JoystickButton climbDoneButton = new JoystickButton(operatorJoysrick, 1);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    /* Drive Buttons */
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(0);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(1);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(2);
    JoystickButton driverRightBumper = new JoystickButton(driverJoystick, 3);
    // Drive Command
    Command driveCommand = new DriveCommand(
      m_swerveSubsystem,
      driverLeftStickX,
      driverLeftStickY,
      driverRightStickX,
      driverRightBumper
    );
    // Set Default Command
    m_swerveSubsystem.setDefaultCommand(driveCommand);
    /* Trun Robot Test */
    
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
