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
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "Amp");
  }
  public boolean isOurAmp(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "SpeakerCenter");
  }
  public boolean isOurSource(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "SpeakerCenter");
  }

  public double[] AimingSPEAKER(){
    double[] Output = {0};
    if(isTargetGet() && isOurSpeaker()){ 
      // Get target measurement
      double targetX = getTarget3dPose().getX();
      double targetY = getTarget3dPose().getY();
      double targetYaw = getTargetYaw();
      // PID calculation
      Output[0] = xMovePID.calculate(targetX, 0);
      Output[1] = yMovePID.calculate(targetY, 0);
      Output[2] = -zTurnPID.calculate(targetYaw, 0);
      // // Bounded      
      // Output[0] = Constants.setMaxOutput(xPidOutput, maxXMovepPIDOutput);
      // Output[1] = Constants.setMaxOutput(yPidOutput, maxYMovePIDOutput);
      // Output[2] = Constants.setMaxOutput(yawPidOutput, maxTurnPIDOutput);
    }else{
      // No target in sight
      Output[0] = 0;
      Output[1] = 0;
      Output[2] = 0;
    }
    return Output;
  }

  @Override
  public void periodic() {
    Latestresult = photonCamera.getLatestResult();
    BestTarget = Latestresult.getBestTarget();
    hasTarget = Latestresult.hasTargets();
    targetID = BestTarget.getFiducialId();
    
    SmartDashboard.putBoolean("HasTarget", isTargetGet());
    SmartDashboard.putNumber("TargetID", getBestTagID());
    SmartDashboard.putNumber("TargetX", getTarget3dPose().getX());
    SmartDashboard.putNumber("TargetY", getTarget3dPose().getY());
  }
}