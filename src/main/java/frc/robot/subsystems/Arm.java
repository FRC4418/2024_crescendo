// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonFX armMaster = new TalonFX(20);
  private final TalonFX armSlave = new TalonFX(22);

  private int peakVelocity = 1000;
  private double percentOfPeak = 0.75;

  private double F = (percentOfPeak * 2048) / (peakVelocity * percentOfPeak);
  private double cruiseVelocity = percentOfPeak * peakVelocity;
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Arm() {


    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = 0.4; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output


    armMaster.getConfigurator().apply(talonFXConfigs);
    armSlave.getConfigurator().apply(talonFXConfigs);
    armMaster.setInverted(false);
    armSlave.setInverted(true);

    // armMaster.setSelectedSensorPosition(0);

    // armMaster.config_kF(0, 0.1);
    // armMaster.config_kP(0, 0.06);
    // armMaster.config_kI(0, 0);
    // armMaster.config_kD(0, 0);

    // armMaster.configMotionSCurveStrength(2);

    // armMaster.setNeutralMode(NeutralMode.Brake);
    // armSlave.setNeutralMode(NeutralMode.Brake);

    // armSlave.follow(armMaster);

    // armSlave.setInverted(InvertType.OpposeMaster);

    // armMaster.configMotionAcceleration(cruiseVelocity);

    // armMaster.configMotionCruiseVelocity(cruiseVelocity);

    // armMaster.selectProfileSlot(0, 0);

    resetEncoder();


  }

  public void goToHome(){
        armMaster.setControl(m_request.withPosition(Constants.armPositions.intake).withFeedForward(0.1));
        armSlave.setControl(m_request.withPosition(Constants.armPositions.intake).withFeedForward(0.1));
  }

  public void stop(){
    armMaster.set(0.0);
  }

   public void resetEncoder(){
     armMaster.setPosition(0);
     armSlave.setPosition(0);
  }

  // public double getMasterPos(){
  //   return armMaster.getSelectedSensorPosition();
  // }

  public void goToPosition(double position){
    armMaster.setControl(m_request.withPosition(position).withFeedForward(0.5));
    armSlave.setControl(m_request.withPosition(position).withFeedForward(0.5));
    System.out.println(position);
  }

  
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(armMaster.getPosition());
  }
}