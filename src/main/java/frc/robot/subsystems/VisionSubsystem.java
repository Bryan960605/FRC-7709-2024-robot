// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraType;

public class VisionSubsystem extends SubsystemBase {
  // Camera
  private final PhotonCamera photonCamera;
  
  private final Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private PhotonPipelineResult Latestresult = null;
  private PhotonTrackedTarget BestTarget = null;
  private boolean hasTarget = false;

  private int targetID;

  public VisionSubsystem() {
    photonCamera = new PhotonCamera(CameraType.HD3000);
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
  /* Target Info */
  // Get target id
  public int getTargetID(){
    return targetID;
  }
  // Get Target Alliance
  public boolean isOurAlliance(int ID){
    var color = alliance.get();
    // Blue
    if ((6<=ID && ID<=10) || (14<=ID && ID<=16)) 
      return color==Alliance.Blue;
    // Red
    else if ((1<=ID && ID<=5) || (11<=ID && ID<=13)) 
      return color==Alliance.Red;
    else return false;
  }
  /* Get Target Pose */
  public Transform3d getTarget3dPose(){
    // Units:meter
    return BestTarget.getBestCameraToTarget();
  }
  public double getTargetYaw(){
    return Math.IEEEremainder(BestTarget.getBestCameraToTarget().getRotation().getZ(), Math.PI);
    // return Units.radiansToDegrees(BestTarget.getBestCameraToTarget().getRotation().getZ())-180;
  }

  @Override
  public void periodic() {
    Latestresult = photonCamera.getLatestResult();
    BestTarget = Latestresult.getBestTarget();
    hasTarget = Latestresult.hasTargets();
    if(isTargetGet()){
      targetID = BestTarget.getFiducialId();
      SmartDashboard.putBoolean("HasTarget", true);
      SmartDashboard.putNumber("TargetID", getTargetID());
      SmartDashboard.putNumber("TargetX", getTarget3dPose().getX());
      SmartDashboard.putNumber("TargetY", getTarget3dPose().getY());
      SmartDashboard.putNumber("RawZ", BestTarget.getYaw());
      SmartDashboard.putNumber("RawPoseZ", BestTarget.getBestCameraToTarget().getRotation().getZ());
      SmartDashboard.putNumber("TargetZ", getTargetYaw());
    }else{
      SmartDashboard.putBoolean("HasTarget", false);
    }
  }
}