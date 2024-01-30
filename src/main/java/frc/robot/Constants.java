// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {
  /* Field Objects */
  public enum FieldObject {
    SPEAKER_CENTER,
    BLUE_SPEAKER_SIDE,
    RED_SPEAKER_SIDE,
    AMP,
    SOURCE_INSIDE,
    SOURCE_OUTSIDE,
    Unknown
}
  /* Robot Parameter */
  public static final double robotLength = Units.inchesToMeters(25.5);
  public static final double robotWidth = Units.inchesToMeters(25.5);
  /* Operator IO */
  public static class OperatorConstants {
    public static final double kJoystickDeadband = 0.05;
    public static final int kDriverJoystickrPort = 0;
    public static final int kOperatorJoystickrPort = 1;
  }
  // Logitech Joystick D-Mode
  public static class LogitechJoystickLayout{
    public static final int BTN_X = 1;
    public static final int BTN_A = 2;
    public static final int BTN_B = 3;
    public static final int BTN_Y = 4;
    public static final int BTN_LEFT_BUMPER = 5;
    public static final int BTN_RIGHT_BUMPER = 6;
    public static final int BTN_LEFT_TRIGGER = 7;
    public static final int BTN_RIGHT_TRIGGER = 8;
    public static final int BTN_BACK = 9;
    public static final int BTN_START = 10;
    public static final int BTN_LEFT_JOYSTICK = 11;
    public static final int AXIS_LEFT_X = 0;
    public static final int AXIS_LEFT_Y = 1;
    public static final int AXIS_RIGHT_X = 2;
    public static final int AXIS_RIGHT_Y = 3;
  }
  /* Camera Type */
  public static class CameraType{
    public static final String HD3000 = "Microsoft_LifeCam_HD-3000";
    public static final String Limelight = "";
  }
  /* Motor Controller CAN ID */
  public static final class MotorControllerIDs{
    // Intake-Shooter
    public static final int kShooterMotorLeftID = 3;
    public static final int kShooterMotorRightID = 4; 
    public static final int kIntakeMotorID = 5;
    // Climber
    public static final int kClimberMotorLeftID = 5;
    public static final int kClimberMotorRightID = 6;
    // Arm
    public static final int kArmMotorLeftID = 1;
    public static final int kArmMotorRightID = 2;
  }
  /* Digital Input */
  public static final class DigitalInputPin{
    public static final int kIRsensorDIO = 0;
    public static final int kLeftClimiberLimitSwitch = 1;
    public static final int kRightClimiberLimitSwitch = 2;
  }
  /* PWM Output */
  public static final class PWMOutputPin{
    public static final int kLeftClimberServo = 0;
    public static final int kRightClimberServo = 1;  
  }
  /* Arm Constants */
  public static final class ArmConstants{
    public static final double armGearRatio = 1/0.0;
    public static final double armOrigin = 0.908;
    public static final double armTaking = 0.0;
    public static final double kArmMaxOutput = 0.3;
    public static final double KArmEncoderOffset = 0;
    // Feedforward
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    // PID Constants
    public static final double kp = 0;
    public static final double ki = 0;
    public static final double kd = 0;
    // Set Angle Degree
    public static final double idleInsideAngle = 60;
    public static final double IdleOutsideAngle = 20;
    public static final double ampAngle = 92;
    public static final double groundIntakeAngle = 0;
  }
  /* Intak-Shooter Constants */
  public static final class IntakeShooterConstants{
    // Shooter PID
    public static final double kp = 0.0;
    public static final double ki = 0.0;
    public static final double kd = 0.0;
    // Constants
    public static final double kSpeakerBaseVolt = 5;
    public static final double kSpeakerRPM = 4000;
    public static final double kAmpBaseVolt = 4;
    public static final double kAmpRPM = 300;
  }
  /* Swerve Module Constants */
  public static final class SwerveModuleConstants{
    public static final double wheelDiameter = Units.inchesToMeters(4);
    public static final double driveMotorGearRatio = 1/6.75;
    public static final double turningMotorGearRatio = 1.0/(150/7);
    public static final double driveEncoderRot2Meter = driveMotorGearRatio*Math.PI*wheelDiameter;
    public static final double turningEncoderRot2Rad = turningMotorGearRatio*2*Math.PI;
    public static final double driveEncoderRPM2MeterPerSec = driveEncoderRot2Meter/60.0;
    public static final double turningEncoderRPM2RadPerSec = turningEncoderRot2Rad/60.0;
    public static final double turningMotorkP = 0.01;

    public static final double maxDriveMotorSpeed = 0.5;
  }
  /* Swerve Drivebase Constants */
  public static final class SwerveConstants{
    public static final int gyroID = 33;
    public static final int leftFrontDriveID = 24;
    public static final int leftFrontTurningID = 23;
    public static final int rightFrontDriveID = 15;
    public static final int rightFrontTurningID = 22;  
    public static final int leftRearDriveID = 11;
    public static final int leftRearTurningID = 16;
    public static final int rightRearDriveID = 17;
    public static final int rightRearTurningID = 21;

    public static final int leftFrontCANCoderID = 1;
    public static final int rightFrontCANCoderID = 2;
    public static final int leftRearCANCoderID = 3;
    public static final int rightRearCANCoderID = 4;

    public static final boolean leftFrontdriveMotorReversed = true;
    public static final boolean rightFrontDriveMotorReversed = false;
    public static final boolean leftRearDriveMotorreversed = false;
    public static final boolean rightRearDriveMotorReversed = false;

    public static final boolean leftFrontTurningMotorReversed = true;
    public static final boolean rightfrontTurningMotorReversed = true;
    public static final boolean leftRearTurningMotorReversed = true;
    public static final boolean rightRearTurningMotorReversed = true;
    
    public static final double leftFrontOffset = -0.461;
    public static final double rightFrontOffset = -0.34;
    public static final double leftRearOffset = 0.151;
    public static final double rightRearOffset = -0.379;

    // public static final double maxWheelSpeed = 0.5;

    //front left, front right, rear left, rear right
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(robotLength/2, robotWidth/2), 
      new Translation2d(robotLength/2, -robotWidth/2), 
      new Translation2d(-robotLength/2, robotWidth/2),
      new Translation2d(-robotLength/2, -robotWidth/2)
    );
  }
  public static double setMaxOutput(double value, double maxOutput){
    if(value > maxOutput) return maxOutput;
    else if(-value < -maxOutput) return -maxOutput;
    else return value;
  }
  /* Aiming Setpoints */
  public static final class AimingSetpoint{
    // Speaker
    public static final double[] Speaker_Center_Setpoint = {0, 0, 0};
    public static final double[] Blue_Speaker_Side_Setpoint = {0, 0, 0};
    public static final double[] Red_Speaker_Side_Setpoint = {0, 0, 0};
    // Amp
    public static final double[] Amp_Setpoint = {0, 0, 0};
    // Source
    public static final double[] Source_Inside_Setpoint = {0, 0, 0};
    public static final double[] Source_Outside_Setpoint = {0, 0, 0};
    // Setpoint Selector
    public static double[] setpointSelector(FieldObject targetPosition){
      switch(targetPosition){
        case SPEAKER_CENTER:
          return Speaker_Center_Setpoint;
        case RED_SPEAKER_SIDE:
          return Red_Speaker_Side_Setpoint;
        case BLUE_SPEAKER_SIDE:
          return Blue_Speaker_Side_Setpoint;
        case AMP:
          return Amp_Setpoint;
        case SOURCE_INSIDE:
          return Source_Inside_Setpoint;
        case SOURCE_OUTSIDE:
          return Source_Outside_Setpoint;
        default:
          return null;
      }
    }
  }
  /* April Tag Constants */
  public static final class ApriltagIDs{
    public static final int RedSourceInside = 1;
    public static final int RedSourceOutside = 2;
    public static final int RedSpeakerSide = 3;
    public static final int RedSpeakerCenter = 4;
    public static final int RedAMPID = 5;

    public static final int BlueAMPID = 6;
    public static final int BlueSpeakerCenter = 7;
    public static final int BlueSpeakerSide = 8;
    public static final int BlueSourceOutside = 9;
    public static final int BlueSourceInside = 10;
    // Target Selector
    public static FieldObject getIDType(int id) {
      switch (id) {
        case ApriltagIDs.RedSpeakerCenter:
        case ApriltagIDs.BlueSpeakerCenter:
          return FieldObject.SPEAKER_CENTER;
        case ApriltagIDs.RedSpeakerSide:
          return FieldObject.RED_SPEAKER_SIDE;
        case ApriltagIDs.BlueSpeakerSide:
          return FieldObject.BLUE_SPEAKER_SIDE;
        case ApriltagIDs.RedAMPID:
        case ApriltagIDs.BlueAMPID:
          return FieldObject.AMP;
        case ApriltagIDs.RedSourceInside:
        case ApriltagIDs.BlueSourceInside:
          return FieldObject.SOURCE_INSIDE;
        case ApriltagIDs.RedSourceOutside:
        case ApriltagIDs.BlueSourceOutside:
          return FieldObject.SOURCE_OUTSIDE;
        default:
          return FieldObject.Unknown;
      }
    }
    public static final double speakerZSetpoint = 0.0;
    public static final double speakerHeight = 204.5;//cm
  }
  
}
