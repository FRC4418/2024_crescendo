// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.AutoStuff;


import java.util.concurrent.ExecutionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.VisionSubsystem;
import frc.utils.TrajectoryUtils;
import frc.utils.VisionUtils;

public class AutoShoot extends Command {
  private Shooter shooter;
  private Intake intake;
  private Arm arm;
  private VisionSubsystem visionSubsystem; 
  private DriveSubsystem driveSubsystem;

  private boolean shooting = false;

  private Timer timer;



  final double ANGULAR_P = 0.1;
  final double ANGULAR_D = 0;
  PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
  

  /** Creates a new autoAngle. */
  public AutoShoot(Shooter shooter, Intake intake ,Arm arm, VisionSubsystem visionSubsystem, DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    timer = new Timer();

    this.arm = arm;
    this.visionSubsystem = visionSubsystem;
    this.shooter = shooter;
    this.intake = intake;
    this.driveSubsystem = driveSubsystem;
    try{
    addRequirements(arm,intake,shooter,visionSubsystem,driveSubsystem);}
    catch(Exception e){

    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("rot speed: " +getRotSpeed() + " arm current pos: " + arm.getArmPos() + " arm desired pos: " + arm.desiredMotorRot);

    driveSubsystem.drive(0, 0, getRotSpeed()/8.4, false, true);

    shooter.spin(1);

    var latestResult = visionSubsystem.getLatestResult();

    if(latestResult == null) return;

    var subTarget = VisionUtils.getId(latestResult, 4, 7);

    if (subTarget == null) return;

    double distToSp = VisionUtils.getDist(subTarget.getPitch());
    double desiredAngle = TrajectoryUtils.getGoodShootingAngle(distToSp);

    System.out.println(desiredAngle);
    if(desiredAngle > arm.lowestAngleDeg) return;
    arm.gotoShooterAngle(desiredAngle);

    if(shooting)     intake.spin(1);


    if(!isInRange(getRotSpeed(),0,0.3)) return; //if its still moveing dont shoot

    if(!isInRange(arm.getArmPos(), arm.desiredMotorRot, 0.3)) return; //if the are is close to pos

    if(!isInRange(shooter.getSpeed(), 80, 18)) return;


    if(!shooting) timer.start();

    shooting = true;
  }


  private boolean isInRange(double num, double target, double range){
    double max = target + range;
    double min = target - range;

    if(num > max) return false;
    if(min > num) return false;
    return true;
  }


  private double getRotSpeed(){
    var result = visionSubsystem.getLatestResult();



    if(result == null) return 0;

    if(!result.hasTargets()){
      return 0;
    }

    double yaw;

    try{yaw = VisionUtils.getId(result, 4, 7).getYaw();
    }catch(Exception e){return 0;}

    return turnController.calculate(yaw, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.3){
      timer = new Timer();
      //shooting = false;
      return true;
    }
    return false;
  }
}
