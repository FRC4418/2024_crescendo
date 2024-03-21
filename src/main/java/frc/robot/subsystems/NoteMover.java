// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteMover extends SubsystemBase {
  public Shooter shooter;
  public Intake intake;

  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  /** Creates a new NoteMover. */
  public NoteMover(Shooter shooter, Intake intake) {
    this.shooter = shooter;
    this.intake = intake;


    var talonFXConfigs = new TalonFXConfiguration();
    var slot0Configs = talonFXConfigs.Slot0;

    slot0Configs.kP = 0.65; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0.4; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
