// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RumbleForTime;
import frc.robot.commands.Intake.IntakeMove;

public class Intake extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(10);
  private TalonFX motor2 = new TalonFX(11);
  public DigitalInput beamBreak = new DigitalInput(5);
  final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  private boolean prevUpdateValue = true;

  private boolean noteSet = false;


  private Vibrator vibrator;

  /** Creates a new shooter. */
  public Intake(Vibrator vibrator) {
    this.vibrator = vibrator;
  }

  public void spin(double speed){
    motor1.set(speed);
    motor2.set(speed);
  }

  public void spinWBeamBreak(double speed){
    SmartDashboard.putBoolean("beam break", beamBreak.get());
    //if(beamBreak.get() == false) spin(speed);
    spin(speed);
    //if(beamBreak.get()) new WaitCommand(0.2).andThen(new IntakeMove(this, 0.1, -.2)).schedule();;
    /*
    if(!beamBreak.get()){ 
      spin(speed);
    }else{
      if(!noteSet){
        //new IntakeMove(this, 0.1, -0.2).schedule();
      }
      noteSet = true;
    }*/
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
    // System.out.println(beamBreak.get());
    boolean current = beamBreak.get();
    
    if(current != prevUpdateValue){ 
      new RumbleForTime(vibrator, RumbleType.kBothRumble, 1, 0.2).schedule();

      
    }


    prevUpdateValue = current;


  }
}
