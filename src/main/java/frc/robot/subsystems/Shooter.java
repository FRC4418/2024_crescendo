// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Rollers. */ 
  private final TalonFX rollerMotor1;
  private final TalonFX rollerMotor2;
  public Shooter() {
    // Replace device number
    rollerMotor1 = new TalonFX(1);
    rollerMotor2 = new TalonFX(2);
    rollerMotor1.setInverted(false);
    rollerMotor2.setInverted(false);
  }

  public void spinRollers(double speed){
    rollerMotor1.set(speed);
    rollerMotor2.set(speed);
  }
    
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Roller Motor Encoder", rollerMotor1.getSelectedSensorPosition());
  }
}
