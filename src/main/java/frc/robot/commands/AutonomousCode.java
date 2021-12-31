// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
//import com.analog.adis16448.frc.*;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import java.util.function.Consumer;
import frc.robot.RobotContainer;

import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.Constants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
//import io.github.pseudoresonance.pixy2api.Pixy2CCC.Block;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/** An example command that uses an example subsystem. */
public class AutonomousCode extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AutonomousSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCode(AutonomousSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.driving);
  }
  public static boolean aPressed = false;
  public static boolean autoMode = false;
  	//public static ADIS16448_IMU imu = new ADIS16448_IMU();
	//public static Ultrasonic ult = new Ultrasonic(1, 1);// Set Actual Input and Output values
	static SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motors Intiaited
  static SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
	//static Encoder rEncoder;
	//static Encoder lEncoder;
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

	public static void testAutonomous(){
		leftmg.setInverted(true);
		boolean aBtn = Constants.logitech.getAButton();

		if (aBtn && !aPressed && !autoMode) {// set auto mode if aBtn is pressed and it is not in auto mode
			aPressed = true;
			autoMode = true;
		//	step = 0;// step counter for future additions to autonomous
		}
		if (aBtn && !aPressed && autoMode) {// turn off autoMode if aButton is pressed, and it wasn't pressed on the
											// prev
			autoMode = false;
			aPressed = true;
			//toDoInProgress = false;

		}
		if (!aBtn) {
			aPressed = false; //a pressed on previous frame
		}

		//if (autoMode) {
		//if(true){
		//	rotateTo(0);
		//}
	//}
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
