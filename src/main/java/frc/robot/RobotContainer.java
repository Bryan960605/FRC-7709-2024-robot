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
import frc.robot.Constants.LogitechJoystickLayout;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnLeftTest;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
  //Subsystem
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
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
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_X);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_Y);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_RIGHT_X);
    // Buttons
    JoystickButton fieldOrientedBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_LEFT_BUMPER);
    JoystickButton turnBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_B);
    // Drive Command
    Command driveCommand = new DriveCommand(
      m_swerveSubsystem,
      driverLeftStickX,
      driverLeftStickY,
      driverRightStickX,
      fieldOrientedBtn
    );
    // Set Default Command
    m_swerveSubsystem.setDefaultCommand(driveCommand);
    /* Trun Robot Test */
    turnBtn.whileTrue(new TurnLeftTest(m_swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
