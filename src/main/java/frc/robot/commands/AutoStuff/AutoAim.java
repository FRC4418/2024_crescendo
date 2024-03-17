// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.AutoStuff;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.VisionUtils;

public class AutoAim extends Command {
  /** Creates a new autoAim. */
  private DriveSubsystem driveSubsystem;
  private VisionSubsystem visionSubsystem;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

  public AutoAim(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.visionSubsystem = visionSubsystem;
    
    addRequirements(driveSubsystem,visionSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.autoSpinSpeed = getRotSpeed()/8.4;
  }

  private double getRotSpeed(){
    var result = visionSubsystem.getLatestResult();

    if(!result.hasTargets()){
      return 0;
    }

    return turnController.calculate(VisionUtils.getId(result, 4, 7).getYaw(), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
