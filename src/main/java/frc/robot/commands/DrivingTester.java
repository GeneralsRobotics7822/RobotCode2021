// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Main;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;

/** An example command that uses an example subsystem. */
public class DrivingTester extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   */
  public DrivingTester(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driving);
  }
 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
    SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
/*
    if (Constants.rt<= -0.5){
        Robot.driving.StandardDrive(leftmg, rightmg);
        System.out.print("forward");
    }
    if (Constants.lt>=0.5){
      Robot.driving.StandardReverse(leftmg, rightmg);
      System.out.print("back");
    }
    if (Constants.lt<0.5){
      Robot.driving.stop();
      System.out.print("stopped");
    }
    if (Constants.rt> -0.5){
      Robot.driving.stop();
      System.out.print("stopped");
    }
    */
    
    if(Constants.yButton){
      Robot.driving.StandardDrive(leftmg, rightmg);
        System.out.print("forward");
    }
    if(Constants.aButton){
      Robot.driving.StandardReverse(leftmg, rightmg);
      System.out.print("back");
    }
    if(Constants.xButton){
      Robot.driving.StandardLeft(leftmg, rightmg);
      System.out.print("left");
    }

    if(Constants.bButton){
      Robot.driving.StandardRight(leftmg, rightmg);
      System.out.print("right");
    }

    if(Constants.rBumper){
      Robot.driving.stop();
      System.out.print("stopped");
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
