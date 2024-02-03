// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new vision. */

  public PhotonCamera camera = new PhotonCamera("pi");

  private PhotonPipelineResult latestResult;

  public VisionSubsystem() {

  }

  public PhotonPipelineResult getLatestResult(){
    return latestResult;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latestResult = camera.getLatestResult();
  }
}
