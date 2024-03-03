// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.AutoStuff;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;

public class IntakeDriveAuto extends Command {
  private Intake intake;
  private DriveSubsystem driveSubsystem;
  /** Creates a new IntakeDriveAuto. */
  public IntakeDriveAuto(DriveSubsystem driveSubsystem, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.driveSubsystem = driveSubsystem;
    try{
    addRequirements(driveSubsystem, intake); }catch(Exception e){}
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
