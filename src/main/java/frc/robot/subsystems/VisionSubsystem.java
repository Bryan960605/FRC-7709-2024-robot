// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.ApriltagConstants.*;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {}
  private final PhotonCamera photonLimelight = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  private final PIDController yMovePID = new PIDController(0.005, 0, 0);
  private final PIDController xMovePID = new PIDController(0.0030, 0, 0);
  private final PIDController turnPID = new PIDController(0.005, 0, 0);

  private final Optional<Alliance> alliance = DriverStation.getAlliance();

  public double yMovePIDOutput, xMovePIDOutput, turnPIDOutput;

  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  public double botXValue;
  private double botYValue;
  private double botZValue;
  private double xSetpoint;
  private double ySetpoint;
  private double zSetpoint;
  private int targetID;
  @Override
  public void periodic() {
    
    PhotonPipelineResult result = photonLimelight.getLatestResult();
    PhotonTrackedTarget target = result.getBestTarget();
    boolean hasTarget = result.hasTargets();

    if(hasTarget){
      botXValue = target.getBestCameraToTarget().getX()*100;
      botYValue = target.getBestCameraToTarget().getY();
      if(alliance.get() == DriverStation.Alliance.Red){
        if(targetID == redSpeakerID1 || targetID == redSpeakerID2){
          botZValue = target.getYaw();
          xSetpoint = 0;
          ySetpoint = 0;
          zSetpoint = speakerZSetpoint;
        }
        else if(targetID == redAMPID){
          botZValue = target.getBestCameraToTarget().getRotation().getAngle();
          xSetpoint = ampXSetpoint;
          ySetpoint = ampYSetpoint;
          zSetpoint = ampZSetpoint;
        }
      }
      else{
        if(targetID == blueSpeakerID1 || targetID == blueSpeakerID2){
          botZValue = target.getYaw();
        }
        else if(targetID == blueAMPID){
          botZValue = target.getBestCameraToTarget().getRotation().getAngle();
        }
      }
    }
    else{
      botXValue = 0;
      botYValue = 0;
      botZValue = 0;
      xSetpoint = 0;
      ySetpoint = 0;
    }
    
    yMovePIDOutput = yMovePID.calculate(botYValue, xSetpoint);
    xMovePIDOutput = xMovePID.calculate(botXValue, ySetpoint);
    turnPIDOutput = -turnPID.calculate(botZValue, zSetpoint);

    xMovePIDOutput = Constants.setMaxOutput(xMovePIDOutput, maxXMovepPIDOutput);
    yMovePIDOutput = Constants.setMaxOutput(yMovePIDOutput, maxYMovePIDOutput);
    turnPIDOutput = Constants.setMaxOutput(turnPIDOutput, maxTurnPIDOutput);
   
    SmartDashboard.putNumber("photonZ", botZValue);
    SmartDashboard.putNumber("photonY", botYValue);
    SmartDashboard.putNumber("photonX", botXValue);
    SmartDashboard.putNumber("targetID", targetID);

    SmartDashboard.putNumber("xMovePIDOutput", xMovePIDOutput);
    SmartDashboard.putNumber("yMovePIDOutput", yMovePIDOutput);
    SmartDashboard.putNumber("turn", turnPIDOutput);
  }
}