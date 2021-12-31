// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import java.lang.Object;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.*;
//import com.analog.adis16448.frc.*;
import java.util.ArrayList;
import java.util.function.Consumer;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.DriveSubsystem;


/** An example command that uses an example subsystem. */
public class AutonomousCodeTinkering extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final AutonomousSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public AutonomousCodeTinkering(AutonomousSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }
  public static boolean autoMode = false;
	//public static ADIS16448_IMU imu = new ADIS16448_IMU();
	//public static Ultrasonic ult = new Ultrasonic(1, 1);// Set Actual Input and Output values
	static SpeedControllerGroup leftmg = new SpeedControllerGroup(DriveSubsystem.lmotor1, DriveSubsystem.lmotor2);//Two Groups of Motors Intiaited
  static SpeedControllerGroup rightmg = new SpeedControllerGroup(DriveSubsystem.rmotor1, DriveSubsystem.rmotor2);
	//static Encoder rEncoder;
  //static Encoder lEncoder;
  


  private static void rotateBy(double targetDeg) 
	{
//First approach to setting direction, should defenitely be
												// improved after testing
												
		double currentDeg = toTerminalAngle(targetDeg);
		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
			leftmg.set(0);
			rightmg.set(0);
			// currentTaskDone = true;
			// return;
		}
		if (currentDeg > 180) {
			double degRot = 360 - currentDeg;
			// System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(-Math.max(-degRot / 500, -0.5));
			rightmg.set(Math.min(degRot / 500, 0.5));
		} else {
			double degRot = currentDeg;
			// System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(-Math.min(degRot / 500, 0.5));
			rightmg.set(Math.max(-degRot / 500, -0.5));
		}
		
	}
	static boolean sAPressed = false;
	static boolean sSlowmode = false;

	// private static void sarahs_tinker() {
	// 	boolean abtn = OI.logitech.getAButton();
	// 	if (abtn && !sAPressed) {
	// 		sAPressed = true;
	// 		sSlowmode = true;
	// 	}
	// 	if (abtn && sAPressed){
	// 		sAPressed = false;
	// 		sSlowmode =false; 
	// 	}
	// }
	
	private static void rotateDrive(double targetDeg) 
	{
//First approach to setting direction, should defenitely be
												// improved after testing
												
		double currentDeg = toTerminalAngle(targetDeg);
		// SmarthDasboard.putNumber("Yaw", imu.getYaw());
		// SmartDashboard.putNumber("XAngle", imu.getAngleX());
		// SmartDashboard.putNumber("YAngle", imu.getAngleY());
		// SmartDashboard.putNumber("ZAngle", imu.getAngleZ());

		if (currentDeg < 5 || currentDeg > 355) {// If it is within 5 degrees, stop
			leftmg.set(0);
			rightmg.set(0);
			// currentTaskDone = true;
			// return;
		}
		if (currentDeg > 180) {
			double degRot = 360 - currentDeg;
			// System.out.println("Rotate " + degRot + " in CCW dir.");
			leftmg.set(-Math.max(-degRot / 500, -0.5));
			rightmg.set(Math.min(degRot / 500, 0.5));
		} else {
			double degRot = currentDeg;
			// System.out.println("Rotate " + degRot + " in CW dir.");
			leftmg.set(-Math.min(degRot / 500, 0.5));
			rightmg.set(Math.max(-degRot / 500, -0.5));
		}
		
	}
	
//public static double getHatchAngle() {}

		// Complete w/ PixyCam
		// Test Pixy Code

		// if (!SmartDashboard.getString("DB/String 2", "null").equals("Running")) {
		// SmartDashboard.putString("DB/String 2", "Running");

		// }

	// 	ArrayList<Block> blocks = RobotMap.pixy.getCCC().getBlocks();
	// 	if (!SmartDashboard.getString("DB/String 1", "null").equals("" +
	// 	blocks.size())) {
	// 	SmartDashboard.putString("DB/String 1", "" + blocks.size());

	// 	}
	// 	if (blocks.size() >= 2) {// Intial version, stands to be improved
	// 		Block leftBlock;
	// 		Block rightBlock;
	// 		if (blocks.get(0).getX() < blocks.get(1).getX()) {
	// 		leftBlock = blocks.get(0);
	// 		rightBlock = blocks.get(1);
	// 		} else {
	// 		leftBlock = blocks.get(1);
	// 		rightBlock = blocks.get(0);
	// 		}
	// 		return (30 / 158 * (blocks.get(0).getX() + blocks.get(1).getX()) / 2 - 158);
	// 		//Uses Pixy2 FOV of 30 deg and resolution of 316 to calculate approx Angle
	// 	}
	// 	return Double.MAX_VALUE;// return Max Value if there is not two blocks

	// }

	private static double toTerminalAngle(double angle) {// Converts to Terminal Angle
		return (angle % 360 + 360) % 360;
	}
	


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
