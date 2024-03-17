// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private TalonFX motorMaster = new TalonFX(21);
  private TalonFX motorSlave = new TalonFX(22);

  private final VelocityVoltage voltageVelocity = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
  
  private Vibrator vibrator;

  /** Creates a new shooter. */
  public Shooter(Vibrator vibrator) {
    this.vibrator = vibrator;

    TalonFXConfiguration fxConfig = new TalonFXConfiguration();
    var currentLimits = new CurrentLimitsConfigs();
    currentLimits.StatorCurrentLimitEnable = true;
    currentLimits.StatorCurrentLimit = 10;
    fxConfig.CurrentLimits = currentLimits;

    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    
    motorMaster.getConfigurator().apply(talonFXConfigs);

    motorSlave.setControl(new Follower(motorMaster.getDeviceID(), false));
  }

  

  public void spin(double speed){
    motorMaster.set(speed);
  }

  public void spinVelocity(double velocity){
    motorMaster.setControl(voltageVelocity.withVelocity(velocity));
  }

  public double getSpeed(){
    return (motorMaster.getVelocity().getValueAsDouble() + motorSlave.getVelocity().getValueAsDouble())/2;
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
    
    SmartDashboard.putNumber("motor 1 (lower) vel:", motorMaster.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("motor 2 (upper) vel:", motorSlave.getVelocity().getValueAsDouble());
    ///r//rrrrrrrrrrrrrrrrrrrrrrrrrr///////////\nnnnnnnnnnnnn\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\.''''''''''''''''''''''''''''''''''\\\.\.\System.out.println(motor1.getVelocity().getValueAsDouble());
  }
}
