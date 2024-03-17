// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.AutoStuff;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AutoSpin;
import frc.robot.subsystems.DriveSubsystem;

public class AutoAimAt extends Command {
  /** Creates a new autoAim. */
  private AutoSpin autoSpin;
  private Supplier<Double> rotSupplier;
  public double desiredPos;

  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);



  public AutoAimAt(AutoSpin autoSpin, Supplier<Double> rotSupplier, double desiredPos) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.autoSpin = autoSpin;
    this.rotSupplier = rotSupplier;
    this.desiredPos = desiredPos;

    
    addRequirements(autoSpin);  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //driveSubsystem.autoSpinSpeed = getRotSpeed()/8.4;
    autoSpin.setSpin(getRotSpeed()/8.4);
  }

  private double getRotSpeed(){

    return turnController.calculate(rotSupplier.get(), desiredPos);
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
