// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class IntakeMove extends Command {
  private double speed;
  private double time;
  private Timer timer;
  private Intake intake;
  /** Creates a new intakeMove. */
  public IntakeMove(Intake intake, double time, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.time = time;
    this.speed = speed;
    timer = new Timer();
    addRequirements(intake);
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
    intake.spin(-.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.spin(0);
    timer.reset();
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
