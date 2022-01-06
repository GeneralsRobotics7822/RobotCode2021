// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Main;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class DriveHub extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  
  public DriveHub(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driving);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialized");
  }
  public static boolean slowMode = false;
  public static boolean superMode = false;
  public static boolean yPressed = false;
  public static boolean xPressed = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!AutonomousCode.autoMode){
      SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
      SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
      leftmg.setInverted(true);//Flip left motor to get it going the right direction
  
      boolean yBtn = Constants.logitech.getYButton();
      boolean xBtn = Constants.logitech.getXButton();//Get states of X and Y button and save them
      
      //slow-mode for demos
      //Robot.driving.RichardDrives(leftmg, rightmg, .25*OI.logitech.getRawAxis(1), -.25*OI.logitech.getRawAxis(4));
      
      if(yBtn&&!yPressed&&!slowMode){//set slow mode if yBtn is pressed and it is not in slow mode
        yPressed=true;
        slowMode=true;
      }
      if(yBtn&&!yPressed&&slowMode){//turn off slowMode if yButton is pressed, and it wasn't pressed on the prev
        slowMode=false;
        yPressed=true;
      }
      if(!yBtn){
        yPressed =false;
      }
  
      if(xBtn&&!xPressed&&!superMode){//set super mode if xBtn is pressed and it is not in super mode
        xPressed=true;
        superMode=true;
      }
      if(xBtn&&!xPressed&&superMode){//turn off superMode if xButton is pressed, and it wasn't pressed on the prev
        superMode=false;
        xPressed=true;
      }
      if(!xBtn){
        xPressed =false;
      }
      if(superMode){//Super Mode Command
        Robot.driving.SarahDrives(leftmg, rightmg, Constants.logitech.getRawAxis(1), -Constants.logitech.getRawAxis(4));
      }
      else if(slowMode){//More sensitive driving
        Robot.driving.SarahDrives(leftmg, rightmg, .2*Constants.logitech.getRawAxis(1), -.2*Constants.logitech.getRawAxis(4));
      }else{//Normal Driving
       Robot.driving.SarahDrives(leftmg, rightmg, .4*Constants.logitech.getRawAxis(1), -.4*Constants.logitech.getRawAxis(4));
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.driving.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
