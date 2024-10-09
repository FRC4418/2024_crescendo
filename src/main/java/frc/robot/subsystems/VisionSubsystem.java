// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.VisionUtils;

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
    latestResult = camera.getLatestResult();

    //SmartDashboard.putNumber("hi", VisionUtils.getDist(VisionUtils.getId(latestResult, 4, 7).getPitch()));
    //System.out.println(latestResult.getBestTarget().getPitch());
    var target = VisionUtils.getId(latestResult, 4, 7);  /// change da 5 to 4 for comp
    if (target == null) return;
    System.out.println(Units.metersToInches(VisionUtils.getDist(target.getPitch())));
  }
}
