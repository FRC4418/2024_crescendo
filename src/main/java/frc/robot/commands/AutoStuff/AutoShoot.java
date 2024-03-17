// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commads.AutoStuff;


import java.util.concurrent.ExecutionException;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public double yaw;



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
    
    addRequirements(arm,intake,shooter,visionSubsystem,driveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooting = false;
    timer = new Timer();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    driveSubsystem.drive(0, 0, getRotSpeed()/6, false, true); //whenever this command is running you should be pointed at tag

    if(shooting)     intake.spin(1);        //if the shooting variable is true intake the note to shoot

    // Max rpms
    shooter.spinVelocity(6380);   //reving up the shooter for when we want to shoot

    var latestResult = visionSubsystem.getLatestResult();   //get the latest result I know we do this later, but this is just for testing if we are in a shootable range

    if(latestResult == null) return;    //if no target no shoot

    var subTarget = VisionUtils.getId(latestResult, 4, 7);   //filter to only shooting april tags

    if (subTarget == null) return;    //if none of those tags are in target no shoot

    double distToSp = VisionUtils.getDist(subTarget.getPitch());        //use class i made to get the distance to sp
    double desiredAngle = TrajectoryUtils.getGoodShootingAngle(distToSp);         //get the desiterd shooter angle

    
    if(desiredAngle > arm.lowestAngleDeg) return;     //if this angle breaks the arm dont
    arm.gotoShooterAngle(desiredAngle);            //if the angle is good go to the angle


    boolean[] checks = new boolean[3];      // make a array for all the checks, you probably could get away with 3 variables but oh well

    checks[0] = isInRange(yaw,0,4);    //if we are within 4 degrees of target then this check is good
    checks[1] = isInRange(arm.getArmPos(), arm.desiredMotorRot, 0.35);  //if the arm is in the correct position we are good
    checks[2] = isInRange(shooter.getSpeed(), 70, 10);   //if the shooter's spinning fast enough we are good

    SmartDashboard.putNumber("yaw: ", yaw);
    SmartDashboard.putNumber("arm error: ", arm.getArmPos() - arm.desiredMotorRot);   //puting all that data into smart dashboard
    SmartDashboard.putNumber("shooter speed", shooter.getSpeed());

    SmartDashboard.putBoolean("check 1", checks[0]);
    SmartDashboard.putBoolean("check 2", checks[1]);
    SmartDashboard.putBoolean("check 3", checks[2]);
    
    

    for(boolean check : checks){  //if any of these are false don't go below this line
      if (!check) return;
    }    //below here, all checks are good



    if(!shooting) {   //if we aren't shooting then start shooting
      shooting = true;

      timer.start();
    }

  }


  private boolean isInRange(double num, double target, double range){  //self explanitory function to help with the checks
    double max = target + range;
    double min = target - range;

    if(num > max) return false;
    if(min > num) return false;
    return true;
  }


  private double getRotSpeed(){   //function for spinning the robot to an april tag
    var result = visionSubsystem.getLatestResult();



    if(result == null) return 0;

    if(!result.hasTargets()){
      return 0;
    }

    

    try{yaw = VisionUtils.getId(result, 4, 7).getYaw();
    }catch(Exception e){return 0;}

    return turnController.calculate(yaw, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {  //when we finish shooting stop the timer and reset
    timer.reset();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.4){          //when the timer for shooting is done end command
      timer = new Timer();          //the reason im not using beam break here is if we miss the note during auto, the entire auto isn't ruined
      //shooting = false;
      return true;
    }
    return false;
  }
}
