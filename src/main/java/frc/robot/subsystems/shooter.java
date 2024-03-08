// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(10);
  private TalonFX motor2 = new TalonFX(11);

  /** Creates a new shooter. */
  public Shooter() {}

  public void spin(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public double getSpeed(){
    return motor1.getVelocity().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This mepthod will be called once per scheduler run
    ///r//rrrrrrrrrrrrrrrrrrrrrrrrrr///////////\nnnnnnnnnnnnn\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\.''''''''''''''''''''''''''''''''''\\\.\.\System.out.println(motor1.getVelocity().getValueAsDouble());
  }
}
