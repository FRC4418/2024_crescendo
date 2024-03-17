// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoStuff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterSpinTime extends Command {
  private Timer timer;
  private double time;
  private Shooter shooter;
  /** Creates a new ShooterSpinTime. */
  public ShooterSpinTime(Shooter shooter, double time) {
    this.shooter = shooter;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.restart();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spin(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spin(0);
    timer.restart();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(timer.get() > time){
      return true;
    } else {
      return false;
    }
  }
}
