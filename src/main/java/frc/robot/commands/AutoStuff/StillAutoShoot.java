// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoStuff;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class StillAutoShoot extends Command {
  private Shooter shooter;
  private Intake intake;
  private Timer timer;
  private Timer timer2;
  private boolean shooting;
  
  /** Creates a new StillAutoShoot. */
  public StillAutoShoot(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;
    addRequirements(shooter, intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting = false;
    timer = new Timer();
    timer.start();
    timer2 = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(shooting) intake.spin(1);

    if(timer.get() > 1.5) {
      if(shooting == false){
        shooting = true;
        timer2.start();
      }
    }

    shooter.spin(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer2.get() > 0.3;
  }
}
