/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.FollowCommandGroup;
import frc.robot.commands.GyroDriveStraightCommand;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.PIDDriveCommand;
import frc.robot.commands.PIDGyroTurnCommand;
import frc.robot.commands.PIDVisionTurnCommand;
import frc.robot.commands.UpdatePIDDriveCommand;
import frc.robot.commands.UpdatePIDVisionCommand;
import frc.robot.commands.ZeroDriveEncodersCommand;
import frc.robot.commands.ZeroGyroCommand;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  public Joystick driveStick = new Joystick(RobotMap.driveJoystickPort);
  public Button aButton = new JoystickButton(driveStick, 1);
  public Button bButton = new JoystickButton(driveStick, 2);
  public Button xButton = new JoystickButton(driveStick, 3);
  public Button yButton = new JoystickButton(driveStick, 4);
  public Button leftBumperButton = new JoystickButton(driveStick, 5);
  public Button rightBumperButton = new JoystickButton(driveStick, 6);
  public Button backButton = new JoystickButton(driveStick, 7);
  public Button startButton = new JoystickButton(driveStick, 8);
  public Button upPOV = new POVButton(driveStick, 0);
  public Button rightPOV = new POVButton(driveStick, 90);
  public Button downPOV = new POVButton(driveStick, 180);
  public Button leftPOV = new POVButton(driveStick, 270);


  public OI(){
 

    SmartDashboard.putData("Zero Encoders", new ZeroDriveEncodersCommand());
    SmartDashboard.putData("PID Drive", new PIDDriveCommand());
    SmartDashboard.putData("Manual Drive", new ManualDriveCommand());
    
    backButton.whenPressed(new ZeroDriveEncodersCommand());
    //button2Button.whenPressed(new ManualDriveCommand());
    //rightBumperButton.whenPressed(new UpdatePIDDriveCommand());
    rightBumperButton.whenPressed(new UpdatePIDVisionCommand());
    xButton.whileHeld(new PIDVisionTurnCommand());
    aButton.whenPressed(new ManualDriveCommand());

    yButton.whileHeld(new PIDGyroTurnCommand(0.0));
    startButton.whenPressed(new ZeroGyroCommand());
    
    leftBumperButton.whenPressed(new FollowCommandGroup());
    upPOV.whileHeld(new GyroDriveStraightCommand());
 
  }
  //// CREATING BUTTONS
  // One type of button is a joystick button which is any button on a
  //// joystick.
  // You create one by telling it which joystick it's on and which button
  // number it is.
  // Joystick stick = new Joystick(port);
  // Button button = new JoystickButton(stick, buttonNumber);

  // There are a few additional built in buttons you can use. Additionally,
  // by subclassing Button you can create custom triggers and bind those to
  // commands the same as any other Button.

  //// TRIGGERING COMMANDS WITH BUTTONS
  // Once you have a button, it's trivial to bind it to a button in one of
  // three ways:

  // Start the command when the button is pressed and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenPressed(new ExampleCommand());

  // Run the command while the button is being held down and interrupt it once
  // the button is released.
  // button.whileHeld(new ExampleCommand());

  // Start the command when the button is released and let it run the command
  // until it is finished as determined by it's isFinished method.
  // button.whenReleased(new ExampleCommand());
}
