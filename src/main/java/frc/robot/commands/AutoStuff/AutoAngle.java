// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoStuff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.TrajectoryUtils;
import frc.utils.VisionUtils;

public class AutoAngle extends Command {
  private Arm arm;
  private VisionSubsystem visionSubsystem; 
  

  /** Creates a new autoAngle. */
  public AutoAngle(Arm arm, VisionSubsystem visionSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.visionSubsystem = visionSubsystem;
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var latestResult = visionSubsystem.getLatestResult();
    var subTarget = VisionUtils.getId(latestResult, 4, 7);

    if (subTarget == null) return;

    double distToSp = VisionUtils.getDist(subTarget.getPitch());
    double desiredAngle = TrajectoryUtils.getGoodShootingAngle(distToSp);

    System.out.println(desiredAngle);
    if(desiredAngle > arm.lowestAngleDeg) return;
    arm.gotoShooterAngle(desiredAngle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
