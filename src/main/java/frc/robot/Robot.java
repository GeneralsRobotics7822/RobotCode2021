// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
  SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(leftmg, rightmg);
  private Command m_autonomousCommand;
  public static DriveSubsystem driving = new DriveSubsystem();
  public static Timer autoTimer = new Timer(); 
  private RobotContainer m_robotContainer;
  static long autoDriveTime; 
  public static XboxController logitech = new XboxController(0);
  public static Victor intake = Constants.intakeMotor;
  public static Victor elevator= Constants.elevatorMotor;
  public static Victor shooter= Constants.shootMotor;



  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
   // CameraServer.getInstance().startAutomaticCapture(0);
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
    autoTimer.reset();
    autoTimer.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    CommandScheduler.getInstance().run();
      // Drive for 2 seconds
      if (autoTimer.get() < 2.0) {
        m_robotDrive.arcadeDrive(-0.5, 0.0); // drive forwards half speed
      } else {
        m_robotDrive.stopMotor(); // stop robot
      }
  }

  @Override
  public void teleopInit() {
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    CommandScheduler.getInstance().run();
    m_robotDrive.arcadeDrive(logitech.getY(Hand.kLeft)*.5, logitech.getX(Hand.kLeft)*.5);

    if (logitech.getY(Hand.kRight)==-1){
      intake.setSpeed(0);
      elevator.setSpeed(-0.6);
    }
    if (logitech.getY(Hand.kRight)==1){
      elevator.setSpeed(0.6);
    }
    if (logitech.getX(Hand.kRight)==-1){
      intake.setSpeed(-0.6);
      elevator.setSpeed(0);
      shooter.setSpeed(0);

    }
    if (logitech.getX(Hand.kRight)==1){
      intake.setSpeed(0);
      elevator.setSpeed(0);
      shooter.setSpeed(-0.75);
    }

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
