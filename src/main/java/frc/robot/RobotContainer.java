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
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LogitechJoystickLayout;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveAimingTarget;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnLeftTest;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  //Subsystem
  private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  // Chooser
  private final SendableChooser<Command> autoChooser;
  //Joystick
  public static final XboxController operatorJoysrick = new XboxController(OperatorConstants.kOperatorJoystickrPort);
  public static final XboxController driverJoystick = new XboxController(OperatorConstants.kDriverJoystickrPort);

  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    configureBindings();
  }

  private void configureBindings() {
    /* Drive Buttons */
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_Y);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_X);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_RIGHT_X);
    // Buttons
    JoystickButton notfieldOrientedBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_LEFT_BUMPER);
    JoystickButton turnBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_B);
    JoystickButton AimingBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_RIGHT_BUMPER);
    JoystickButton ZeroingGyroBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_X);
    // Drive Command
    Command driveCommand = new DriveCommand(
      m_swerveSubsystem,
      driverLeftStickX,
      driverLeftStickY,
      driverRightStickX,
      !notfieldOrientedBtn.getAsBoolean()
    );
    // Set Default Command
    m_swerveSubsystem.setDefaultCommand(driveCommand);
    /* Trun Robot Test */
    turnBtn.whileTrue(new TurnLeftTest(m_swerveSubsystem));
    /* Aiming */
    AimingBtn.whileTrue(new DriveAimingTarget(m_swerveSubsystem, m_VisionSubsystem));
    /* Reset Gyro */
    ZeroingGyroBtn.onTrue(
      new RunCommand(
        ()->{m_swerveSubsystem.resetGyro();}
    ));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
