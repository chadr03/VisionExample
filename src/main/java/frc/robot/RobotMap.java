/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

    //Joystick Ports, Axis, and Buttons
    public static int driveJoystickPort = 0;
    public static int driveMoveAxis = 1;
    public static int driveTurnAxis = 4;


    //PWM Ports
    public static int left1Port = 2; //2
    public static int left2Port = 3; //3
    public static int right1Port = 0; //0
    public static int right2Port = 1; //1
    public static int climbLeftPort = 4; //4 needs +1
    public static int climbRightPort = 7; //7 needs +1
    public static int bottomIntakeMotorPort = 5; //5 +1 sucks
    public static int topIntakeMotorPort = 6; //6 -1 sucks
  
    //DIO Ports
    public static int rightEncoderA = 0;
    public static int rightEncoderB = 1;
    public static int leftEncoderA = 4;
    public static int leftEncoderB = 5;

    //Various Constants
    public static final double DRIVE_ENCODER_PULSE_PER_FT =  0.000766990394;  //for 6" wheel and 2048 pulse per rev encoder
}
