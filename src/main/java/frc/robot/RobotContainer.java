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
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commads.Intake.IntakeNote;
import frc.robot.commads.Intake.IntakeShoot;
import frc.robot.commads.Intake.IntakeSpit;
import frc.robot.commads.Shooter.spinShooter;
import frc.robot.commads.Arm.ArmDown;
import frc.robot.commads.Arm.ArmToPosition;
import frc.robot.commads.Arm.ArmUp;
import frc.robot.commads.AutoStuff.Aim;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Shooter;
import frc.utils.AutoCommandBuilder;
import frc.utils.AutoUtils;
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
  private final Intake intake = new Intake();
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter();
  private boolean fieldRelative = true;
  //private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);
  CommandXboxController m_CommandXboxControllerManipulator = new CommandXboxController(1);

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
    m_CommandXboxControllerManipulator.povUp().onTrue(new ArmToPosition(arm, Constants.armPositions.Position1));
    m_CommandXboxControllerManipulator.povLeft().onTrue(new ArmToPosition(arm, Constants.armPositions.Position2));
    m_CommandXboxControllerManipulator.povRight().onTrue(new ArmToPosition(arm, Constants.armPositions.Position3));
    m_CommandXboxControllerManipulator.povDown().onTrue(new ArmToPosition(arm, Constants.armPositions.Position4));

    m_CommandXboxControllerManipulator.b().whileTrue(new IntakeSpit(intake));

    m_CommandXboxControllerManipulator.leftTrigger().whileTrue(new spinShooter(shooter, -1));
    m_CommandXboxControllerManipulator.rightTrigger().whileTrue(new spinShooter(shooter, 1));

    m_CommandXboxControllerManipulator.x().onTrue(new InstantCommand(() -> {
      if (intake.beamBreak.get()) { new IntakeShoot(intake);
      } else { new IntakeNote(intake); } }));

    m_CommandXboxControllerManipulator.leftBumper().whileTrue(new ArmDown(arm));
    m_CommandXboxControllerManipulator.rightBumper().whileTrue(new ArmUp(arm));



    m_CommandXboxControllerDriver.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading()));
    m_CommandXboxControllerDriver.x().onTrue(new InstantCommand(() -> fieldRelative = !fieldRelative)
    .andThen(new RunCommand(
      () -> m_robotDrive.drive(
        -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
        -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
        fieldRelative, true), // pass fieldRelative as the last argument
      m_robotDrive)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    //return AutoUtils.getCommandFromPathName("New Path", m_robotDrive);
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);
    AutoBuilder.addPath("New Path", true);
    AutoBuilder.addPath("New New Path", false);
    AutoBuilder.addPath("New Path", false);    
    AutoBuilder.addPath("New New Path", false);
    AutoBuilder.addPath("New Path", false);    
    AutoBuilder.addPath("New New Path", false);
    return AutoBuilder.getAuto();
  }
}
