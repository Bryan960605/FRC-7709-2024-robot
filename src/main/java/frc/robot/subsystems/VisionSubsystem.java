// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ApriltagIDs;
import frc.robot.Constants.CameraType;

public class VisionSubsystem extends SubsystemBase {
  // Camera
  private final PhotonCamera photonCamera = new PhotonCamera(CameraType.HD3000);
  
  private final Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private PhotonPipelineResult Latestresult;
  private PhotonTrackedTarget BestTarget;
  private boolean hasTarget;

  /* PID Controller */
  private final PIDController yMovePID;
  private final PIDController xMovePID;
  private final PIDController zTurnPID;

  private final double maxXMovepPIDOutput = 0.3; 
  private final double maxYMovePIDOutput = 0.3;
  private final double maxTurnPIDOutput = 0.5;

  private int targetID;

  public VisionSubsystem() {
    yMovePID = new PIDController(0.005, 0, 0);
    xMovePID = new PIDController(0.0030, 0, 0);
    zTurnPID = new PIDController(0.005, 0, 0);
  }

  public boolean isBlueAlliance(){
    return (alliance.get()==DriverStation.Alliance.Blue) ? true : false;
  }
 
  public boolean isTargetGet(){
    return hasTarget;
  }

  public PhotonPipelineResult getLatestResult(){
    return Latestresult;
  }

  public PhotonTrackedTarget getBestTarget(){
    return BestTarget;
  }

  public int getBestTagID(){
    return targetID;
  }

  public Transform3d getTarget3dPose(){
    // Units:meter
    return BestTarget.getBestCameraToTarget();
  }

  public double getTargetYaw(){
    return BestTarget.getYaw();
  }

  public boolean isOurSpeaker(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "SpeakerCenter");
  }

  public double[] AimingSPEAKER(){
    double xPidOutput, yPidOutput, yawPidOutput;
    double[] Output = {0};
    if(isTargetGet() && isOurSpeaker()){ 
      // Get target measurement
      double targetX = getTarget3dPose().getX();
      double targetY = getTarget3dPose().getY();
      double targetYaw = getTargetYaw();
      // PID calculation
      xPidOutput = xMovePID.calculate(targetX, 0);
      yPidOutput = yMovePID.calculate(targetY, 0);
      yawPidOutput = -zTurnPID.calculate(targetYaw, 0);
      // Bounded      
      Output[0] = Constants.setMaxOutput(xPidOutput, maxXMovepPIDOutput);
      Output[1] = Constants.setMaxOutput(yPidOutput, maxYMovePIDOutput);
      Output[2] = Constants.setMaxOutput(yawPidOutput, maxTurnPIDOutput);
    }else{
      // No target in sight
      xPidOutput = 0;
      yPidOutput = 0;
      yawPidOutput = 0;
    }
    return Output;
  }

  @Override
  public void periodic() {
    Latestresult = photonCamera.getLatestResult();
    BestTarget = Latestresult.getBestTarget();
    hasTarget = Latestresult.hasTargets();
    targetID = BestTarget.getFiducialId();

    // if(hasTarget){
    //   botXValue = BestTarget.getBestCameraToTarget().getX()*100;
    //   botYValue = BestTarget.getBestCameraToTarget().getY();
    //   // Red Alliance
    //   if(alliance.get() == DriverStation.Alliance.Red){
    //     if(targetID == redSpeakerID1 || targetID == redSpeakerID2){
    //       botZValue = BestTarget.getYaw();
    //       xSetpoint = 0;
    //       ySetpoint = 0;
    //       zSetpoint = speakerZSetpoint;
    //     }
    //     else if(targetID == redAMPID){
    //       botZValue = BestTarget.getBestCameraToTarget().getRotation().getAngle();
    //     }
    //   }
    //   else{
    //     if(targetID == blueSpeakerID1 || targetID == blueSpeakerID2){
    //       botZValue = BestTarget.getYaw();
    //     }
    //     else if(targetID == blueAMPID){
    //       botZValue = BestTarget.getBestCameraToTarget().getRotation().getAngle();
    //     }
    //   }
    // }
    // else{
    //   botXValue = 0;
    //   botYValue = 0;
    //   botZValue = 0;
    //   xSetpoint = 0;
    //   ySetpoint = 0;
    // }
    
   
    // SmartDashboard.putNumber("Yaw", botZValue);
    // SmartDashboard.putNumber("photonY", botYValue);
    // SmartDashboard.putNumber("photonX", botXValue);
    // SmartDashboard.putNumber("targetID", targetID);
  }
}