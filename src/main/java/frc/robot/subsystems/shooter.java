// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(10);
  private TalonFX motor2 = new TalonFX(11);

  private Vibrator vibrator;

  /** Creates a new shooter. */
  public Shooter(Vibrator vibrator) {
    this.vibrator = vibrator;

    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = 10;
    fxConfig.CurrentLimits = currentLimits;
  }

  public void spin(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public double getSpeed(){
    return (motor1.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble())/2;
  }

  @Override
  public void periodic() {
    // This mepthod will be called once per scheduler run

    if(getSpeed() > 62){
      System.out.println(getSpeed());
      vibrator.controller2.setRumble(RumbleType.kLeftRumble, 1);
    }else{
      vibrator.controller2.setRumble(RumbleType.kBothRumble, 0);
    }
    
    SmartDashboard.putNumber("motor 1 (lower) vel:", motor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("motor 2 (upper) vel:", motor2.getVelocity().getValueAsDouble());
    ///r//rrrrrrrrrrrrrrrrrrrrrrrrrr///////////\nnnnnnnnnnnnn\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\.''''''''''''''''''''''''''''''''''\\\.\.\System.out.println(motor1.getVelocity().getValueAsDouble());
  }
}
