// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Intake.IntakeDumb;
import frc.robot.commands.Intake.IntakeMove;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Intake.IntakeShoot;
import frc.robot.commands.Intake.IntakeSpin;
import frc.robot.commands.Shooter.ShooterMoveForTime;
import frc.robot.commands.Shooter.spinShooter;
import frc.robot.commands.RumbleForTime;
import frc.robot.commands.WinchCommand;
import frc.robot.commands.Arm.ArmDown;
import frc.robot.commands.Arm.ArmToPosition;
import frc.robot.commands.Arm.ArmToPositionAuto;
import frc.robot.commands.Arm.ArmUp;
import frc.robot.commands.Arm.ShooterToAngle;
import frc.robot.commands.Arm.ArmSet;
import frc.robot.commands.AutoStuff.Aim;
import frc.robot.commands.AutoStuff.AutoAimAtEnding;
import frc.robot.commands.AutoStuff.AutoAngle;
import frc.robot.commands.AutoStuff.AutoShoot;
import frc.robot.commands.AutoStuff.ShooterSpinTime;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.Winch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vibrator;
import frc.utils.AutoCommandBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {


  
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_otherController = new XboxController(1);
  CommandXboxController m_CommandXboxControllerDriver = new CommandXboxController(0);
  CommandXboxController m_CommandXboxControllerManipulator = new CommandXboxController(1);

  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();


  private final Vibrator vibrator = new Vibrator(m_driverController, m_otherController);

  private final Intake intake = new Intake(vibrator);
  private final Arm arm = new Arm();
  private final Shooter shooter = new Shooter(vibrator);
  private final Winch winch = new Winch();
  private final VisionSubsystem m_VisionSubsystem = new VisionSubsystem();


  private SendableChooser<Command> chooser = new SendableChooser<>();


  

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    boolean flipped = false;
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      flipped = alliance.get() == DriverStation.Alliance.Red;
    }

    chooser.setDefaultOption("one note", oneNote());
    chooser.addOption("amp side 2p", ampSide2p(flipped));
    chooser.addOption("center 2p", center2p(flipped));
    chooser.addOption("open side 2p", openSide2p(flipped));

    chooser.addOption("center 4p", center4p(flipped));
    chooser.addOption("center 3p", center3p(flipped));

    chooser.addOption("open side far 2p", openSideFar2p(flipped));

    chooser.addOption("test", test(flipped));

    SmartDashboard.putData(chooser);
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
    m_CommandXboxControllerManipulator.povLeft().onTrue(new ArmToPosition(arm, 23));

    m_CommandXboxControllerManipulator.povRight().onTrue(new ShooterToAngle(arm, 0));

    m_CommandXboxControllerManipulator.leftBumper().whileTrue(new ArmDown(arm));

    m_CommandXboxControllerManipulator.rightBumper().whileTrue(new ArmUp(arm));

    m_CommandXboxControllerManipulator.b().whileTrue(new IntakeMove(intake, 0.1, -.2));

    m_CommandXboxControllerManipulator.rightTrigger().whileTrue(new spinShooter(shooter, 6380));

    m_CommandXboxControllerManipulator.a().whileTrue(new IntakeDumb(intake, 1) );

    m_CommandXboxControllerManipulator.x().whileTrue(new IntakeSpin(intake, 1));

    m_CommandXboxControllerManipulator.povDown().onTrue(new ArmToPosition(arm, 0));

    m_CommandXboxControllerManipulator.povUp().whileTrue(new ArmUp(arm));

    m_CommandXboxControllerManipulator.y().whileTrue(new InstantCommand(() -> arm.resetEncoder()));

    m_CommandXboxControllerManipulator.leftTrigger().toggleOnTrue(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    

    m_CommandXboxControllerDriver.a().onTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(0)));

    m_CommandXboxControllerDriver.b().onTrue(new IntakeMove(intake, 0.1, -0.2).alongWith(new ShooterMoveForTime(shooter, -1, 0.1)));

    m_CommandXboxControllerDriver.x().whileTrue(new IntakeDumb(intake, 1));

    m_CommandXboxControllerDriver.rightTrigger().whileTrue(new spinShooter(shooter, 6380));

    m_CommandXboxControllerDriver.leftTrigger().toggleOnTrue(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));


  


    
    m_CommandXboxControllerDriver.y().whileTrue(new InstantCommand(() -> arm.resetEncoder()));

    //m_CommandXboxControllerDriver.rightBumper().whileTrue(new AutoAngle(arm, m_VisionSubsystem).alongWith(new Aim(m_robotDrive, m_VisionSubsystem)));

    m_CommandXboxControllerDriver.rightBumper().toggleOnTrue(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    //m_CommandXboxControllerDriver.leftBumper().onTrue(new ShooterToAngle(arm, 0));
    m_CommandXboxControllerDriver.leftBumper().whileTrue(new WinchCommand(winch, 0.2) );

    m_CommandXboxControllerDriver.povLeft().whileTrue(new WinchCommand(winch, -0.2));

    //m_CommandXboxControllerDriver.povUp().onTrue(new ArmToPosition(arm, 23));
    m_CommandXboxControllerDriver.povDown().onTrue(new ArmToPosition(arm, 0));


    arm.setDefaultCommand(new ArmSet(arm, 0));
    intake.setDefaultCommand(new IntakeDumb(intake, 0));
    shooter.setDefaultCommand(new spinShooter(shooter, 0));
    vibrator.setDefaultCommand(new RumbleForTime(vibrator, RumbleType.kBothRumble, 0, 0.1));
    winch.setDefaultCommand(new WinchCommand(winch, 0) );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    boolean flipped = false;
    var alliance = DriverStation.getAlliance();

    if (alliance.isPresent()) {
      flipped = alliance.get() == DriverStation.Alliance.Red;
    }

    //return AutoUtils.getCommandFromPathName("New Path", m_robotDrive);
    
    //return goto0

    //return mid2p(flipped);

    return chooser.getSelected();
  }

  public Command test(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    AutoBuilder.addPath("center to amp note near", true, flipped);

    return AutoBuilder.getAuto();
  }

  public Command openSideFar2p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);
    AutoBuilder.setSpeeds(3, 2);
    
    AutoBuilder.addRace("open side to open note far", true, flipped, new IntakeSpin(intake, 0.6));

    AutoBuilder.addPair("open note far to shooting pt1", false, flipped, new IntakeMove(intake, 0.1, -0.3));

    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    return AutoBuilder.getAuto();
  }

  public Command center3p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);


    AutoBuilder.addRace("center to amp note near", true, flipped, new IntakeDumb(intake, 0.5));
    
    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
    AutoBuilder.addCommand(new InstantCommand(()-> arm.spin(0)));
    AutoBuilder.addCommand(new InstantCommand(  ()-> shooter.spin(0)  ));
    AutoBuilder.addCommand(new InstantCommand(  ()-> intake.spin(0)  ));

    AutoBuilder.setSpeeds(2, 2);

    AutoBuilder.addPath("amp note near to amp note center", false, flipped);

    AutoBuilder.setSpeeds(1, 1);

    AutoBuilder.addRace("intake center note near", false, flipped, new IntakeDumb(intake, 0.5));

    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    return AutoBuilder.getAuto();
  }

  public Command center4p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    AutoBuilder.addRace("center to amp note near", true, flipped, new IntakeDumb(intake, 0.5));
    
    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
    AutoBuilder.addCommand(new InstantCommand(()-> arm.spin(0)));
    AutoBuilder.addCommand(new InstantCommand(  ()-> shooter.spin(0)  ));
    AutoBuilder.addCommand(new InstantCommand(  ()-> intake.spin(0)  ));

    AutoBuilder.setSpeeds(2, 2);

    AutoBuilder.addPath("amp note near to amp note center", false, flipped);

    AutoBuilder.setSpeeds(1, 1);

    AutoBuilder.addRace("intake center note near", false, flipped, new IntakeDumb(intake, 0.5));

    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
    AutoBuilder.addCommand(new InstantCommand(()-> arm.spin(0)));
    AutoBuilder.addCommand(new InstantCommand(  ()-> shooter.spin(0)  ));
    AutoBuilder.addCommand(new InstantCommand(  ()-> intake.spin(0)  ));

    AutoBuilder.setSpeeds(2, 2);

    AutoBuilder.addPath("amp note center to amp note far", false, flipped);

    AutoBuilder.setSpeeds(1, 1);

    AutoBuilder.addRace("intake amp note far", false, flipped, new IntakeDumb(intake, 0.5));

    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    return AutoBuilder.getAuto();
  }

  public Command openSide2p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);
    
    AutoBuilder.addRace("open side to open note near", true, flipped, new IntakeDumb(intake, 0.5));
    
    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    return AutoBuilder.getAuto();
  }

  public Command center2p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    AutoBuilder.addRace("center to center note near", true, flipped, new IntakeDumb(intake, 0.5));
    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
    return AutoBuilder.getAuto();
  }

  public Command ampSide2p( boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    AutoBuilder.addRace("amp side to amp note near", true, flipped, new IntakeDumb(intake, 0.5));
    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.3));
    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
    //AutoBuilder.addPath("path2", false, flipped);
    return AutoBuilder.getAuto();
  }






  //below here is all not used and useless
  //i should probably just delete




















  public Command rightSideForBlue(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));

    Command shoot = new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3));



    Command rev = new ShooterSpinTime(shooter, 1.5);

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));

    AutoBuilder.addCommand(rev);

    AutoBuilder.addCommand(shoot);

    AutoBuilder.addPath("open side move", true, flipped);

    return AutoBuilder.getAuto();
  }



  public Command mid2p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));


    Command rev = new ShooterSpinTime(shooter, 1.5);

    Command intakeCommand = new IntakeMove(intake, 1, 1d);


    Command outTake = new IntakeMove(intake, 0.1, -0.2);

    Command shoot = new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3));

    Command setupShot = new AutoAngle(arm, m_VisionSubsystem).alongWith(new Aim(m_robotDrive, m_VisionSubsystem));    

    Command goofyShoot = new IntakeMove(intake, 0.3, -1);

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));

    AutoBuilder.addCommand(rev);

    AutoBuilder.addCommand(shoot);

    //AutoBuilder.addPath("New Path", true, true);

    //AutoBuilder.addPath("start to left", true, flipped); //move from starting pose to left near note

    AutoBuilder.addPair("intake from center", true, flipped, new IntakeMove(intake, 2, 1));  //intatke the note

    AutoBuilder.addCommand(outTake);  //outake the note a little pit to not stall the shooter

    AutoBuilder.addPath("mid back to center", false, flipped);

    AutoBuilder.addCommand(new ShooterSpinTime(shooter, 1));

    AutoBuilder.addCommand(new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3)));

    //AutoBuilder.addCommand(setupShot);

    //AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    /*     
    AutoBuilder.addPair("left to mid note", false, flipped, new AutoAimAt(new AutoSpin(m_robotDrive), m_robotDrive::getYawDouble , 0));

    AutoBuilder.addPair("intake mid near", false, flipped, new IntakeMove(intake, 1, 1));

    AutoBuilder.addCommand(new IntakeMove(intake, 0.1, -0.2));

    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));
*/

    return AutoBuilder.getAuto();


    
  }

  public Command left2p(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));
    
    AutoBuilder.addPath("left to left short", true, flipped);

    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    AutoBuilder.addCommand(new AutoAimAtEnding(m_robotDrive, m_robotDrive::getYawDouble, 0d));

    AutoBuilder.addPair("intake left near", false, flipped, new IntakeMove(intake, 1, 1));

    AutoBuilder.addCommand(new AutoShoot(shooter, intake, arm, m_VisionSubsystem, m_robotDrive));

    return AutoBuilder.getAuto();
  }

  public Command oneNote(){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));


    Command rev = new ShooterSpinTime(shooter, 1.5);

    Command intakeCommand = new IntakeMove(intake, 1, 1d);

    Command outTake = new IntakeMove(intake, 0.1, -0.2);

    Command shoot = new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3));

    Command setupShot = new AutoAngle(arm, m_VisionSubsystem).alongWith(new Aim(m_robotDrive, m_VisionSubsystem));    

    Command goofyShoot = new IntakeMove(intake, 0.3, -1);

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));

    AutoBuilder.addCommand(rev);

    AutoBuilder.addCommand(shoot);

    return AutoBuilder.getAuto();
  }

  public Command leftMove(boolean flipped){
    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));


    Command rev = new ShooterSpinTime(shooter, 1.7);

    Command intakeCommand = new IntakeMove(intake, 1, 1d);

    Command outTake = new IntakeMove(intake, 0.1, -0.2);

    Command shoot = new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3));

    Command setupShot = new AutoAngle(arm, m_VisionSubsystem).alongWith(new Aim(m_robotDrive, m_VisionSubsystem));    

    Command goofyShoot = new IntakeMove(intake, 0.3, -1);

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));

    AutoBuilder.addCommand(rev);

    AutoBuilder.addCommand(shoot);

    AutoBuilder.addPath("left mobility", true, flipped);

    return AutoBuilder.getAuto();
  }


  public Command rightMove(boolean flipped){

    AutoCommandBuilder AutoBuilder = new AutoCommandBuilder(m_robotDrive);

    Command zero = new InstantCommand(() -> arm.resetEncoder());


    Command goto0 = new ArmToPositionAuto(arm, -23).andThen( new InstantCommand(() -> arm.spin(0)));


    Command rev = new ShooterSpinTime(shooter, 1.5);

    Command intakeCommand = new IntakeMove(intake, 1, 1d);

    Command outTake = new IntakeMove(intake, 0.1, -0.2);

    Command shoot = new ParallelCommandGroup(new IntakeMove(intake, 0.3, 1), new ShooterSpinTime(shooter, 0.3));

    Command setupShot = new AutoAngle(arm, m_VisionSubsystem).alongWith(new Aim(m_robotDrive, m_VisionSubsystem));    

    Command goofyShoot = new IntakeMove(intake, 0.3, -1);

    AutoBuilder.addCommand(zero);

    AutoBuilder.addCommand(goto0);  

    AutoBuilder.addCommand(new InstantCommand(() -> arm.resetEncoder()));

    AutoBuilder.addCommand(rev);

    AutoBuilder.addCommand(shoot);

    AutoBuilder.addPath("right mobility", true, flipped);

    //AutoBuilder.addPair("test spiny", false, flipped, new AutoAimAt(new AutoSpin(m_robotDrive), m_robotDrive::getYawDouble , 180));

    return AutoBuilder.getAuto();
  }

}
