// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
//import frc.robot.commands.Drive_command;
//import frc.robot.commands.HookCommand;
//import frc.robot.subsystems.HookSubsystem;
//import frc.robot.commands.IntakeToShootCommand;
//import frc.robot.commands.PixyAlignCommand;
//import frc.robot.subsystems.IntakeToShootSubsystem;
//import frc.robot.subsystems.PixyAlignSubsystem;
import frc.robot.subsystems.AutonomousSubsystem;
// import com.revrobotics.ColorSensorV3;
// import com.revrobotics.ColorMatchResult;
// import com.revrobotics.ColorMatch;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveSubsystem;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public static DriveSubsystem driving = new DriveSubsystem();
  private RobotContainer m_robotContainer;
  //public static PixyAlignSubsystem align = new PixyAlignSubsystem();

  //public static IntakeToShootSubsystem intake = new IntakeToShootSubsystem();
  //public static HookSubsystem hook= new HookSubsystem(); 
  public static UsbCamera camera1;
 // public static DigitalInput elevatorLimit= new DigitalInput(1);
 // public static DigitalInput hookLimit= new DigitalInput(2);
  static long autoDriveTime; 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.getInstance().startAutomaticCapture(0);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {
    CommandScheduler.getInstance().run();
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //Robot.align.setDefaultCommand(new PixyAlignCommand());
    //autoDriveTime = System.currentTimeMillis();
    //m_autonomousCommand = m_robotContainer.getAutonomous();
    //Robot.align.setDefaultCommand(new PixyAlignCommand());
    //autoDriveTime = System.currentTimeMillis();
    // schedule the autonomous command (example)
    //if (m_autonomousCommand != null) {
    //  m_autonomousCommand.schedule();
    //}
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
    /*
    if(Math.subtractExact(System.currentTimeMillis(), autoDriveTime)>3000)
    {
     Constants.elevatorMotor.setSpeed(-0.6);
    }
    if(Math.subtractExact(System.currentTimeMillis(), autoDriveTime)>13500)
    {
      */
      //Constants.elevatorMotor.setSpeed(0);
      //Constants.shootMotor.setSpeed(0);
      Constants.leftMotor1.setSpeed(0.25);
      Constants.leftMotor2.setSpeed(0.25);
      Constants.rightMotor1.setSpeed(-0.25);
      Constants.rightMotor2.setSpeed(-0.25);
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //Robot.driving.setDefaultCommand(new Drive_command());
    //Robot.intake.setDefaultCommand(new IntakeToShootCommand());
    //Robot.hook.setDefaultCommand(new HookCommand());
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    LiveWindow.setEnabled(true);
  }
}
