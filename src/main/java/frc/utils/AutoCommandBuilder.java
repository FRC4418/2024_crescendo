// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;

/** Add your docs here. */
public class AutoCommandBuilder{
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    DriveSubsystem m_robotDrive;
    public double maxSpeed = 1;
    public double maxAccel = 1;

    public AutoCommandBuilder(DriveSubsystem m_robotDrive){
        this.m_robotDrive = m_robotDrive;
    }

    public void addPair(String pathName, boolean firstPath, boolean flipped, Command command){
        addCommand(AutoUtils.getCommandFromPathName(pathName, m_robotDrive, firstPath, flipped, maxSpeed, maxAccel).alongWith(command));
    }

    public void addRace(String pathName, boolean firstPath, boolean flipped, Command command){
        addCommand(new ParallelRaceGroup(AutoUtils.getCommandFromPathName(pathName, m_robotDrive, firstPath, flipped, maxSpeed, maxAccel),command));
    }

    public void setSpeeds(double speed, double accel){
        maxSpeed = speed;
        maxAccel = accel;
    }


    public void addCommand(Command command){
        autoCommand.addCommands(command);
    }

    public void addPath(String pathName,boolean firstPath, boolean flipped){
        autoCommand.addCommands(AutoUtils.getCommandFromPathName(pathName, m_robotDrive, firstPath, flipped, maxSpeed, maxAccel));
    }

    public Command getAuto()
    {
        return autoCommand;
    }
}