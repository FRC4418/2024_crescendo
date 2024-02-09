// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Commands.autoAim;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
//import com.stuypulse.stuylib.input.gamepads.AutoGamepad;
import com.pathplanner.lib.path.PathPoint;


import edu.wpi.first.math.trajectory.proto.TrajectoryStateProto;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_CommandXboxController = new CommandXboxController(0);

  //AutoGamepad driver = new AutoGamepad(0);

  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true, true),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    //driver.getBottomButton().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_CommandXboxController.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    // Create config for trajector
/* 
    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
*/

    // Reset odometry to the starting pose of the trajectory.
    //m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
        
    //PathPlannerPath path = PathPlannerPath.fromPathFile("pidTestStopStart");

    //PathPoint point = path.getPoint(0);

    
    
      // Create a path following command using AutoBuilder. This will also trigger event markers.
    //Command movekkkkkkkkkkkkkkkkkkForward = AutoBuilder.followPathWithEvents(path);



    
    //return autoCommand;

    //return new autoAim(m_robotDrive, m_VisionSubsystem);
    


    InstantCommand resetHeading = new InstantCommand(
        () -> m_robotDrive.zeroHeading()
    );

    InstantCommand resetPose = new InstantCommand(
        () -> m_robotDrive.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)))
    );


    return getCommandFromPathName("New Path").andThen(() -> m_robotDrive.drive(0, 0, 0, false, false));

  }

  
  public Trajectory pathPlanerToTrajectory(PathPlannerPath pathPlannerPath, TrajectoryConfig config){

    List<Translation2d> points = new ArrayList<Translation2d>();

    
    List<State> states = new ArrayList<State>();

    List<PathPoint> pathPoints = pathPlannerPath.getAllPathPoints();

    PathPlannerTrajectory ppTraj = pathPlannerPath.getTrajectory(new ChassisSpeeds(), new Rotation2d());
    
    

    for (int i = 0; i < pathPoints.size(); i++) {
      //if (i==0 || i == pathPoints.size()-1) continue;
      PathPoint pathPoint = pathPoints.get(i);             //get all the points
      points.add(pathPoint.position);
    }

    states.add(ppStateToState(ppTraj.getInitialState()));

    for (com.pathplanner.lib.path.PathPlannerTrajectory.State ppState : ppTraj.getStates()){
      edu.wpi.first.math.trajectory.Trajectory.State state = ppStateToState(ppState);
      states.add(state);
    }

    com.pathplanner.lib.path.PathPlannerTrajectory.State ppState = ppTraj.getEndState();

    states.add(ppStateToState(ppState));
    

    // return TrajectoryGenerator.generateTrajectory(
    //     pathPlannerPath.getPreviewStartingHolonomicPose(), 
    //     new ArrayList<Translation2d>(), 
    //     endPose, 
    //     config
    // );

    List<Pose2d> ls = new ArrayList<Pose2d>();
    ls.add(pathPlannerPath.getPreviewStartingHolonomicPose());
    ls.add(new Pose2d(points.get(points.size()-1), new Rotation2d()));


    Trajectory traj = new Trajectory(states);

    

    //return traj;
    return TrajectoryGenerator.generateTrajectory(ls, config);
  }

  public State ppStateToState(com.pathplanner.lib.path.PathPlannerTrajectory.State ppState){
    return new State(ppState.timeSeconds,ppState.velocityMps,ppState.accelerationMpsSq,ppState.getTargetHolonomicPose(),ppState.curvatureRadPerMeter);
  }


  public Command getCommandFromPathName(String pathName){

    //PathPlannerTrajectory ppTraj = PathPlannerPath.fromPathFile(pathName).getTrajectory(new ChassisSpeeds(), new Rotation2d());

    
    var thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
      thetaController.enableContinuousInput(-Math.PI, Math.PI
    );


    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        //Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics
    );

    Trajectory traj = pathPlanerToTrajectory(PathPlannerPath.fromPathFile(pathName), config);

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config
      );

    Command drive = new SwerveControllerCommand(
        traj,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive
    );

    Command resetPose = new InstantCommand(
      () -> m_robotDrive.resetOdometry(traj.getInitialPose())
    );

    InstantCommand resetHeading = new InstantCommand(
      () -> m_robotDrive.zeroHeading()
    );

    // System.out.println("Init pose = " + traj.getInitialPose() + "total time = " + traj.getTotalTimeSeconds() + "states = " + traj.getStates());

    return new SequentialCommandGroup(resetHeading, resetPose, drive);
  }

  public Pose2d getStartingPose(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    return path.getPreviewStartingHolonomicPose();
  }
}
