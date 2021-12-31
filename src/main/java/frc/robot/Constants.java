// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Victor leftMotor1 = new Victor(0);//Motors on their respectives ports
    public static Victor leftMotor2 = new Victor(2);
    public static Victor rightMotor1 = new Victor(1);
    public static Victor rightMotor2 = new Victor(3);
    public static XboxController logitech = new XboxController(0); 

    //public static Victor shootMotor = new Victor(7);
    //public static Victor intakeMotor= new Victor(8);
   // public static Victor elevatorMotor = new Victor(9);
   // public static Victor hookUpMotor = new Victor(6);
    //public static Victor robotUpMotor = new Victor(4);

    // public static George_Pixy pixy = new George_Pixy();//Pixy Cam


}
