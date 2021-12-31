// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Victor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  public static Victor lmotor1= Constants.leftMotor1; //reset
  public static Victor lmotor2= Constants.leftMotor2;
  public static Victor rmotor1 = Constants.rightMotor1;
  public static Victor rmotor2 = Constants.rightMotor2;
  public static SpeedControllerGroup leftmg = new SpeedControllerGroup(lmotor1, lmotor2);
  public static SpeedControllerGroup rightmg = new SpeedControllerGroup(rmotor1, rmotor2);
  
  public DriveSubsystem() {}

  public void squareSpeed(double left, double right)
{
  lmotor2.set(-left*Math.abs(left));
  lmotor1.set(-left*Math.abs(left));
  rmotor2.set(right*Math.abs(right));
  rmotor1.set(right*Math.abs(right));
}

public void speed(double left, double right)
{
  lmotor2.set(-left);
  lmotor1.set(-left);
  rmotor2.set(right);
  rmotor1.set(right);
}

public void stop()
{
  lmotor2.set(0);
  lmotor1.set(0);
  rmotor2.set(0);
  rmotor1.set(0);
}

public void StandardDrive(SpeedControllerGroup lmg, SpeedControllerGroup rmg)
  {
    lmg.set(0.5);
    rmg.set(0.5);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void SarahDrives(SpeedControllerGroup lmg, SpeedControllerGroup rmg, double power, double direction){
    lmg.set(Math.max(-power+direction, -1));
    rmg.set(Math.min(-direction-power,1));
  }

}
