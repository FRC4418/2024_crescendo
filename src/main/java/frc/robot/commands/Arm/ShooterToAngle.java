// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;

public class ShooterToAngle extends Command {
  /** Creates a new ArmIntake. */
  private final Arm arm;
  private double angle;
  private double desiredMotorRot;
  public final double lowestAngleDeg = 48;
  public ShooterToAngle(Arm arm, double angleDeg) {
    this.arm = arm;
    this.angle = angleDeg;
    try{
    addRequirements(arm);  }catch(Exception e){}

    

    double desiredMovementFrom0Deg = lowestAngleDeg - this.angle;
    double desiredMovementFrom0Rot = desiredMovementFrom0Deg/360;
    desiredMotorRot = desiredMovementFrom0Rot * 80 * Math.PI;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //arm.goToPosition(desiredMotorRot);
    arm.gotoShooterAngle(angle);
    //System.out.println(desiredMotorRot);
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
