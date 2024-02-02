package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MountPoseConfigs;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveModuleConstants;

import static frc.robot.Constants.SwerveConstants.*;

import java.util.Optional;


public class SwerveSubsystem extends SubsystemBase{
    /* Swerve Modules */
    private final SwerveModule leftFrontModule, rightFrontModule, leftRearModule, rightRearModule;
    /* Gyro */
    private final Pigeon2 gyro = new Pigeon2(gyroID);
    private final Pigeon2Configuration gyroConfig = new Pigeon2Configuration();
    /* Odometer */
    private SwerveDriveOdometry odometer;
    /* Field 2D */
    private final Field2d field = new Field2d();

    public SwerveSubsystem(){
      /* Gyro Configuration */
      gyroConfig.withMountPose(new MountPoseConfigs().withMountPoseYaw(-90));
      gyro.getConfigurator().apply(gyroConfig);
      /* Create Modules */
      leftFrontModule = new SwerveModule(
        leftFrontDriveID, 
        leftFrontTurningID, 
        leftFrontdriveMotorReversed, 
        leftFrontTurningMotorReversed, 
        leftFrontCANCoderID, 
        leftFrontOffset);
      rightFrontModule = new SwerveModule(
        rightFrontDriveID,
        rightFrontTurningID,
        rightFrontDriveMotorReversed, 
        rightfrontTurningMotorReversed, 
        rightFrontCANCoderID, 
        rightFrontOffset);
      leftRearModule = new SwerveModule(
        leftRearDriveID, 
        leftRearTurningID, 
        leftRearDriveMotorreversed, 
        leftRearTurningMotorReversed, 
        leftRearCANCoderID, 
        leftRearOffset);
      rightRearModule = new SwerveModule(
        rightRearDriveID, 
        rightRearTurningID, 
        rightRearDriveMotorReversed, 
        rightRearTurningMotorReversed, 
        rightRearCANCoderID, 
        rightRearOffset);
      /* Odometer Init */
      odometer = new SwerveDriveOdometry(
          swerveKinematics, 
          gyro.getRotation2d(), 
          getModulePosition());
      /* Field2D init */
      SmartDashboard.putData("Field", field);
      /* Auto Bilder */
      AutoBuilder.configureHolonomic(
        this::getPose, 
        this::setPose, 
        this::getSpeeds, 
        this::drive_auto,
        new HolonomicPathFollowerConfig(
          new PIDConstants(4.5, 0, 0), // Translation constants 
          new PIDConstants(2, 0, 0.002), // Rotation constants 
          SwerveModuleConstants.maxDriveMotorSpeed, 
          Units.inchesToMeters(14.32), // Drive base radius (distance from center to furthest module) 
          new ReplanningConfig()
        ),
        () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          Optional<Alliance> alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this
      );
      // Set up custom logging to add the current path to a field 2d widget
      PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));
    }
    public SwerveModulePosition[] getModulePosition(){
        return new SwerveModulePosition[]{
          leftFrontModule.getPosition(),
          rightFrontModule.getPosition(),
          leftRearModule.getPosition(),
          rightRearModule.getPosition()
        };
    }
    public SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[]{
          leftFrontModule.getState(),
          rightFrontModule.getState(),
          leftRearModule.getState(),
          rightRearModule.getState()
        };
    }
    public void setModuleStates(SwerveModuleState[] desiredStates){
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveModuleConstants.maxDriveMotorSpeed);
      leftFrontModule.setDesiredState(desiredStates[0]);
      rightFrontModule.setDesiredState(desiredStates[1]);
      leftRearModule.setDesiredState(desiredStates[2]);
      rightRearModule.setDesiredState(desiredStates[3]);
    }
    public void stopModules(){
      leftFrontModule.stopModule();
      rightFrontModule.stopModule();
      leftRearModule.stopModule();
      rightRearModule.stopModule();
    }
    public void drive(double xSpeed, double ySpeed, double zSpeed, boolean isFieldOriented){
        SwerveModuleState[] states = null;
        if(isFieldOriented){
            states = swerveKinematics.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getHeading()));
        }else{
            states = swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        }
        SmartDashboard.putNumber("input_x", xSpeed);
        SmartDashboard.putNumber("input_y", ySpeed);
        SmartDashboard.putNumber("input_rotate", zSpeed);
        setModuleStates(states);
    }
    // Auto Drive
    public void drive_auto(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
        SwerveModuleState[] states = swerveKinematics.toSwerveModuleStates(targetSpeeds);
        setModuleStates(states);
    }
    public Pose2d getPose(){
        return odometer.getPoseMeters();
    }
    public void setPose(Pose2d pose){
        odometer.resetPosition(gyro.getRotation2d(), getModulePosition(), pose);
    }
    public ChassisSpeeds getSpeeds() {
      return swerveKinematics.toChassisSpeeds(getModuleStates());
    }
    public Rotation2d getHeading(){
      return gyro.getRotation2d();
    }
    public void resetGyro(){
        gyro.reset();
    }
    @Override
    public void periodic(){
        odometer.update(gyro.getRotation2d(), getModulePosition());
        SmartDashboard.putNumber("RR_DP", leftFrontModule.getDrivePosition());
        SmartDashboard.putNumber("RF_DP", leftRearModule.getDrivePosition());
        SmartDashboard.putNumber("LF_DP", rightFrontModule.getDrivePosition());
        SmartDashboard.putNumber("LR_DP", rightRearModule.getDrivePosition());
        SmartDashboard.putNumber("RR_Angle", rightRearModule.getTurningPosition());
        SmartDashboard.putNumber("RF_Angle", rightFrontModule.getTurningPosition());
        SmartDashboard.putNumber("LF_Angle", leftFrontModule.getTurningPosition());
        SmartDashboard.putNumber("LR_Angle", leftRearModule.getTurningPosition());
        SmartDashboard.putNumber("GyroReading", getHeading().getDegrees());
    }
  
}