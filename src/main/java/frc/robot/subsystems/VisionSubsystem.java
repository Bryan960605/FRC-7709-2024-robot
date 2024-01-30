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
import frc.robot.Constants.AimingSetpoint;
import frc.robot.Constants.ApriltagIDs;
import frc.robot.Constants.CameraType;

public class VisionSubsystem extends SubsystemBase {
  // Camera
  private final PhotonCamera photonCamera = new PhotonCamera(CameraType.HD3000);
  
  private final Optional<Alliance> alliance = DriverStation.getAlliance();
  
  private PhotonPipelineResult Latestresult = null;
  private PhotonTrackedTarget BestTarget = null;
  private boolean hasTarget = false;

  /* PID Controller */
  private final PIDController yMovePID;
  private final PIDController xMovePID;
  private final PIDController zTurnPID;

  private int targetID;

  public VisionSubsystem() {
    yMovePID = new PIDController(0.05, 0, 0);
    xMovePID = new PIDController(0.030, 0, 0);
    zTurnPID = new PIDController(0.005, 0, 0);
  }

  public boolean isBlueAlliance(){
    return alliance.get()==Alliance.Blue;
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

  public boolean isOurAmp(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "Amp");
  }
  public boolean isOurSpeaker(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "SpeakerCenter")?true:false;
  }
  public boolean isOurSource(){
    return getBestTagID()==ApriltagIDs.getApriltagID(isBlueAlliance(), "SourceInside")?true:false;
  }

  public double[] AimingSPEAKER(){
    double[] Output = {0,0,0};
    if(isTargetGet() && isOurSpeaker()){ 
      System.out.println("Aiming SPEAKER");
      // Get target measurement
      double targetX = getTarget3dPose().getX();
      double targetY = getTarget3dPose().getY();
      double targetYaw = getTargetYaw();
      // PID calculation
      Output[0] = xMovePID.calculate(targetX, AimingSetpoint.SPEAKER[0]);
      Output[1] = yMovePID.calculate(targetY, AimingSetpoint.SPEAKER[1]);
      Output[2] = -zTurnPID.calculate(targetYaw, AimingSetpoint.SPEAKER[2]);
      // // Bounded      
      // Output[0] = Constants.setMaxOutput(xPidOutput, maxXMovepPIDOutput);
      // Output[1] = Constants.setMaxOutput(yPidOutput, maxYMovePIDOutput);
      // Output[2] = Constants.setMaxOutput(yawPidOutput, maxTurnPIDOutput);
    }else{
      // No target in sight
      Output[0] = 0;
      Output[1] = 0;
      Output[2] = 0;
      System.out.println("I can't see SPEAKER!");
    }
    return Output;
  }
  public double[] AimingAMP(){
    double[] Output = {0, 0, 0};
    if(isTargetGet()==true && isOurAmp()==true){ 
      System.out.println("Aiming AMP");
      // Get target measurement
      double targetX = getTarget3dPose().getX();
      double targetY = getTarget3dPose().getY();
      double targetYaw = getTargetYaw();
      // PID calculation
      Output[0] = xMovePID.calculate(targetX, AimingSetpoint.AMP[0]);
      Output[1] = yMovePID.calculate(targetY, AimingSetpoint.AMP[1]);
      Output[2] = -zTurnPID.calculate(targetYaw, AimingSetpoint.AMP[2]);
      // // Bounded      
      // Output[0] = Constants.setMaxOutput(xPidOutput, maxXMovepPIDOutput);
      // Output[1] = Constants.setMaxOutput(yPidOutput, maxYMovePIDOutput);
      // Output[2] = Constants.setMaxOutput(yawPidOutput, maxTurnPIDOutput);
    }else{
      // No target in sight
      Output[0] = 0;
      Output[1] = 0;
      Output[2] = 0;
      System.out.println("I can't see AMP!");
    }
    return Output;
  }
  public double[] AimingSOURCE(){
    double[] Output = {0,0,0};
    if(isTargetGet() && isOurSource()){ 
      System.out.println("Aiming SOURCE");
      // Get target measurement
      double targetX = getTarget3dPose().getX();
      double targetY = getTarget3dPose().getY();
      double targetYaw = getTargetYaw();
      // PID calculation
      Output[0] = xMovePID.calculate(targetX, AimingSetpoint.SPEAKER[0]);
      Output[1] = yMovePID.calculate(targetY, AimingSetpoint.SPEAKER[1]);
      Output[2] = -zTurnPID.calculate(targetYaw, AimingSetpoint.SPEAKER[2]);
      // // Bounded      
      // Output[0] = Constants.setMaxOutput(xPidOutput, maxXMovepPIDOutput);
      // Output[1] = Constants.setMaxOutput(yPidOutput, maxYMovePIDOutput);
      // Output[2] = Constants.setMaxOutput(yawPidOutput, maxTurnPIDOutput);
    }else{
      // No target in sight
      Output[0] = 0;
      Output[1] = 0;
      Output[2] = 0;
      System.out.println("I can't see SOURCE!");
    }
    return Output;
  }
  @Override
  public void periodic() {
    Latestresult = photonCamera.getLatestResult();
    BestTarget = Latestresult.getBestTarget();
    hasTarget = Latestresult.hasTargets();
    SmartDashboard.putBoolean("isBlue", isOurAmp());
    SmartDashboard.putNumber("Amp", ApriltagIDs.getApriltagID(isBlueAlliance(), "Amp"));
    if(isTargetGet()){
      targetID = BestTarget.getFiducialId();
      SmartDashboard.putBoolean("HasTarget", isTargetGet());
      SmartDashboard.putNumber("TargetID", getBestTagID());
      SmartDashboard.putNumber("TargetX", getTarget3dPose().getX());
      SmartDashboard.putNumber("TargetY", getTarget3dPose().getY());
    }else{
      SmartDashboard.putBoolean("HasTarget", false);
    }
  }
}