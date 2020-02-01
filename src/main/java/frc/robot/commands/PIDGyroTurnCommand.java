/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;


public class PIDGyroTurnCommand extends Command {
  private double turnAngle;
  private double kP = 0.015;
  private double kD = 0.001;
  private double kI = 0.01;
  private double kF = 0.4;
  
  private double lastTimestamp = Timer.getFPGATimestamp();
  private double errorSum = 0;
  private double lastError = 0;
  private double iZone =  10.0;
  StringBuilder outputString = new StringBuilder(); 
  
  public PIDGyroTurnCommand(double angle) {
    // Use requires() here to declare subsystem dependencies
    requires(Robot.drive);
    turnAngle = angle;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    //errorSum = 0;
    //lastTimestamp = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double error = turnAngle - Robot.drive.gyro.getAngle();
    outputString.append("error: " + String.format("%.3f", error));
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    outputString.append("dt: " + String.format("%.3f", dt));
    kF = Math.copySign(kF, error);

    if(Math.abs(error) < iZone){
      errorSum = errorSum + error * dt;
    }else{
      errorSum = 0;
    }
    String strDouble = String.format("%.2f", 2.00023); 

    double errorRate = (error - lastError)/dt;
    double outF = kF;             //Feed forward output
    outputString.append("Feed Forward: " + String.format("%.3f", outF));
    double outP = kP * error;     //Proportional output
    outputString.append("    Proportional: " + String.format("%.3f",outP));
    double outI = kI * errorSum;  //Intigrator output
    outputString.append(" ersum: " + String.format("%.3f",errorSum));
    outputString.append("    Intigrator: " + String.format("%.3f",outI));
    double outD = kD * errorRate; //Derivitive output
    outputString.append("    Derivitive: " + String.format("%.3f",outD));


    double outputTurn = outF + outP + outI + outD;


    outputString.append("    Total Out: " + String.format("%.3f",outputTurn));
    System.out.println(outputString.toString());
    outputString.setLength(0);
    Robot.drive.teleopDrive(0, outputTurn);

    //SmartDashboard.putNumber("Vision Turn Value", outputTurn);

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;

  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    Robot.drive.teleopDrive(0, 0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    end();
  }
}
