// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter;

public class spinShooter extends Command {
  public shooter shooter;
  public double speed;
  /** Creates a new spinShooter. */
  public spinShooter(shooter shooter, double speed) {
    this.shooter = shooter;
    addRequirements(shooter);
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spin(speed);
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