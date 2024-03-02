// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.path.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class AutoUtils 
{
    public static Trajectory pathPlanerToTrajectory(PathPlannerPath pathPlannerPath, TrajectoryConfig config, boolean flipped)
    {

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
        Pose2d start = pathPlannerPath.getPreviewStartingHolonomicPose();
        Pose2d end = new Pose2d(points.get(points.size()-1), new Rotation2d());
        if(flipped){
            Translation2d pos = start.getTranslation();

            start = new Pose2d(pos.getX(), -pos.getY(), start.getRotation());
            Translation2d pos2 = end.getTranslation();

            end = new Pose2d(pos2.getX(), -pos2.getY(), end.getRotation());
        }

        ls.add(start);
        ls.add(end);


        Trajectory traj = new Trajectory(states);

        //System.out.println(traj.getStates().toString());

        //return traj;
        return TrajectoryGenerator.generateTrajectory(ls, config);
    }

    public static State ppStateToState(com.pathplanner.lib.path.PathPlannerTrajectory.State ppState){
        return new State(ppState.timeSeconds,ppState.velocityMps,ppState.accelerationMpsSq,ppState.getTargetHolonomicPose(),ppState.curvatureRadPerMeter);
    }

    public static Command getCommandFromPathName(String pathName, DriveSubsystem m_robotDrive, boolean firstRun, boolean flipped){

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

        Trajectory traj = pathPlanerToTrajectory(PathPlannerPath.fromPathFile(pathName), config, flipped);

        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d(0, 0.5)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(0, 1, new Rotation2d(0)),
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

        Command resetHeading = new InstantCommand(
        () -> m_robotDrive.zeroHeading()
        );

        Command speedZero = new InstantCommand(
            () -> m_robotDrive.driveRobotRelative(new ChassisSpeeds())
        );

        // System.out.println("Init pose = " + traj.getInitialPose() + "total time = " + traj.getTotalTimeSeconds() + "states = " + traj.getStates());
        if(!firstRun) return drive;
        return new SequentialCommandGroup(resetHeading, resetPose, drive);
    }

    public static Pose2d getStartingPose(String pathName){
        PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
        return path.getPreviewStartingHolonomicPose();
    }
}
