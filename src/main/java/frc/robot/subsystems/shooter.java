// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class shooter extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(12);
  private TalonFX motor2 = new TalonFX(23);

  /** Creates a new shooter. */
  public shooter() {}

  public void spin(double speed){
    motor1.set(speed);
    motor2.set(speed*.8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
