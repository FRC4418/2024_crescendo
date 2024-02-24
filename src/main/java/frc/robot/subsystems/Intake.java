// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(20);
  private TalonFX motor2 = new TalonFX(21);
  public DigitalInput beamBreak = new DigitalInput(0);
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /** Creates a new shooter. */
  public Intake() {}

  public void spin(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void resetEncoders(){
    //motor1.setPosition(0);
    //motor2.setPosition(0);
  }


  public void goToPosition(double position){
    motor1.setControl(m_request.withPosition(position).withFeedForward(0.5));
    motor2.setControl(m_request.withPosition(position).withFeedForward(0.5));
    //System.out.println(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
