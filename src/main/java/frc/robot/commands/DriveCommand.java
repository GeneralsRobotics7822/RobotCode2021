// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem drive_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(DriveSubsystem subsystem) {
    drive_subsystem = subsystem;
    addRequirements(Robot.driving);
    // Use addRequirements() here to declare subsystem dependencies.
    ///addRequirements();
  }

  // Called when the command is initially scheduled.
  public static SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motor Intiaited
  public static SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Robot.driving.SarahDrives(leftmg, rightmg, -0.25*RobotContainer.logitech.getRawAxis(4), 0.25*RobotContainer.logitech.getRawAxis(1));
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
