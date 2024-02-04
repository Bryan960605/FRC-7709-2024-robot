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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.LogitechJoystickLayout;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeNoteGround;
import frc.robot.commands.ShootNote;
import frc.robot.commands.Aiming.DriveAimingTarget;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
  //Subsystem
  private final SwerveSubsystem m_SwerveSubsystem = new SwerveSubsystem();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  // Chooser
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Boolean> debugChooser;
  //Joystick
  public static final XboxController operatorJoysrick = new XboxController(OperatorConstants.kOperatorJoystickrPort);
  public static final XboxController driverJoystick = new XboxController(OperatorConstants.kDriverJoystickrPort);

  public RobotContainer() {
    /* Auto Path Chooser */
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto mode", autoChooser);
    debugChooser = new SendableChooser<>();
    configureBindings();
  }

  public void displayInfo(){
    Constants.globalDebug = debugChooser.getSelected();
  }

  private void configureBindings() {
    /* Drive Buttons */
    DoubleSupplier driverLeftStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_Y);
    DoubleSupplier driverLeftStickY = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_LEFT_X);
    DoubleSupplier driverRightStickX = () -> driverJoystick.getRawAxis(LogitechJoystickLayout.AXIS_RIGHT_X);
    // Buttons
    // JoystickButton notfieldOrientedBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_LEFT_BUMPER);
    JoystickButton AimingBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_RIGHT_BUMPER);
    JoystickButton ZeroingGyroBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_X);
    JoystickButton ShootBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_Y);
    JoystickButton IntakeBtn = new JoystickButton(driverJoystick, LogitechJoystickLayout.BTN_LEFT_BUMPER);
    // Drive Command
    Command driveCommand = new DriveCommand(
      m_SwerveSubsystem,
      driverLeftStickX,
      driverLeftStickY,
      driverRightStickX,
      true
    );
    // Set Default Command
    m_SwerveSubsystem.setDefaultCommand(driveCommand);
    /* Aiming */
    AimingBtn.whileTrue(new DriveAimingTarget(m_SwerveSubsystem, m_VisionSubsystem));
    /* Aiming + Shoot Note */
    ShootBtn.whileTrue(new ShootNote(m_SwerveSubsystem, m_VisionSubsystem, m_ShooterSubsystem));
    /* Intake Note */
    IntakeBtn.toggleOnTrue(new IntakeNoteGround(m_IntakeSubsystem));
    /* Reset Gyro */
    ZeroingGyroBtn.onTrue(
      Commands.runOnce(
        ()->{m_SwerveSubsystem.resetGyro();}
    ));
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
