// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
    final JoystickButton  lTrigger = new JoystickButton(logitech, 6);
    final JoystickButton  rTrigger = new JoystickButton(logitech, 9);
    public static double rt = logitech.getTriggerAxis(GenericHID.Hand.kRight);
    public static double lt = logitech.getTriggerAxis(GenericHID.Hand.kLeft);
    public static boolean aButton = logitech.getAButton();
    public static boolean xButton = logitech.getXButton();
    public static boolean yButton = logitech.getYButton();
    public static boolean bButton = logitech.getBButton();
}

