// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
//import jdk.vm.ci.meta.Constant;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import edu.wpi.first.wpilibj.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.DriveSubsystem;
/** An example command that uses an example subsystem. */

public class AutonomousDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AutonomousSubsystem m_subsystem;
  /**
   * Creates a new ExampleCommand.
   */
  public AutonomousDrive(AutonomousSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
   addRequirements(Robot.auto);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialized");
  }
  public static boolean LeftStart=false;
  public static boolean MiddleStart=false;
  public static boolean RightStart=false;
  public static boolean autoMode=false;
  public static boolean yBtn=false;
  public static boolean pixyMode=false;
  public static Timer timer= new Timer();
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SpeedControllerGroup leftmg = new SpeedControllerGroup(AutonomousSubsystem.lmotor1, AutonomousSubsystem.lmotor2);//Two Groups of Motor Intiaited
    SpeedControllerGroup rightmg = new SpeedControllerGroup(AutonomousSubsystem.rmotor1, AutonomousSubsystem.rmotor2);
    leftmg.setInverted(true);//Flip left motor to get it going the right direction
    //slow-mode for demos
   // Robot.driving.SarahDrives(leftmg, rightmg, .25*Constants.logitech.getRawAxis(1), -.25*Constants.logitech.getRawAxis(4));
   Timer timer = Robot.autoTimer; //timer works in microseconds
   if(timer.get()<5)
   {
   leftmg.set(0.25);
   rightmg.set(0.25);
   System.out.print("forward");
   }
   else
   {
   leftmg.set(0);
   rightmg.set(0);
   System.out.print("stopped");
   }
   }
    // if(LeftStart){
    //   Robot.driving.SarahDrives(leftmg, rightmg, OI.logitech.getRawAxis(1), -OI.logitech.getRawAxis(4));
    // }
    //else if(MiddleStart){
      //Robot.auto.AutonomousStart(leftmg, rightmg);
      //timer.start();
      //if(yBtn)
      //{
     //   pixyMode=true;
     // }
     // Robot.auto.TimedDistance(leftmg, rightmg, Robot.autoTimer);
     // Robot.auto.TimedDistance(leftmg, rightmg);
    //}
    // else if(RightStart)
    // {
    //   Robot.driving.SarahDrives(leftmg, rightmg, 0, -.8*OI.joystick.getRawAxis(1));
    // }
    // else{//Normal Driving
    //  Robot.driving.LeftMotorStrength(leftmg, rightmg);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
