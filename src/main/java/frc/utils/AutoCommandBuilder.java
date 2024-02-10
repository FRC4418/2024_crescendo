// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utils;

import java.util.List;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

import java.util.ArrayList;

/** Add your docs here. */
public class AutoCommandBuilder{
    List<Object> autoCommandUnbuilt = new ArrayList<Object>();
    SequentialCommandGroup autoCommand = new SequentialCommandGroup();
    DriveSubsystem m_robotDrive;

    public AutoCommandBuilder(DriveSubsystem m_robotDrive){
        this.m_robotDrive = m_robotDrive;
    }

    public void addPair(String pathName, Command command){
        addCommand(AutoUtils.getCommandFromPathName(pathName, m_robotDrive).alongWith(command));
    }

    public void addCommand(Command command){
        autoCommandUnbuilt.add(command);
    }

    public void addPath(String pathName){
        autoCommandUnbuilt.add(pathName);
    }

    public String toString(){
        String str = new String();

        for(Object obj : autoCommandUnbuilt){
            str = str + obj.toString() + "\n";
        }

        return str;
    }

    public Command getAuto()
    {
        for (Object obj : autoCommandUnbuilt)
        {
            try
            {
                autoCommand.addCommands(AutoUtils.getCommandFromPathName((String)obj, m_robotDrive));
            } catch(Exception e)
            {
                System.out.println("wassup");
            }

            try 
            {
                autoCommand.addCommands((Command) obj);
            } catch (Exception e) 
            {
                System.out.println("hi");
            }
        }

        return autoCommand;
    }
}