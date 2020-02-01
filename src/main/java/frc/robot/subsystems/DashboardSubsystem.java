/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/**
 * This class is for all dashboard items.  They must initially be placed on the dashboard in the constructor
 * then updated in the periodic method.
 */
public class DashboardSubsystem extends Subsystem {

  public DashboardSubsystem() {
    

    // SmartDashboard.putNumber("Drive PID Target Position", Robot.drive.getTargetPosition());
    // SmartDashboard.putNumber("Drive kP", Robot.drive.getDriveKP());
    // SmartDashboard.putNumber("Drive kI", Robot.drive.getDriveKI());
    // SmartDashboard.putNumber("Drive kD", Robot.drive.getDriveKD());
    // SmartDashboard.putNumber("Drive kF", Robot.drive.getDriveKF());
    // SmartDashboard.putNumber("Output", 0);

    SmartDashboard.putNumber("Turn PID Target Position", Robot.drive.getTargetPosition());
    SmartDashboard.putNumber("Turn kP", Robot.drive.getVisionKP());
    SmartDashboard.putNumber("Turn kI", Robot.drive.getVisionKI());
    SmartDashboard.putNumber("Turn kD", Robot.drive.getVisionKD());
    SmartDashboard.putNumber("Turn kF", Robot.drive.getVisionKF());
    SmartDashboard.putNumber("Vision Turn Value", 0);


  }

  

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Right Encoder Position", Robot.drive.rightEncoderCount());
    SmartDashboard.putNumber("Left Encoder Position", Robot.drive.leftEncoderCount());
    SmartDashboard.putNumber("Right Encoder Speed", Robot.drive.rightEncoderSpeed());
    SmartDashboard.putNumber("Left Encoder Speed", Robot.drive.leftEncoderSpeed());
    SmartDashboard.putNumber("Right Encoder Distance", Robot.drive.rightEncoderDistance());
    SmartDashboard.putNumber("Left Encoder Distance", Robot.drive.leftEncoderDistance());
    SmartDashboard.putNumber("Gyro Angle", Robot.drive.gyro.getAngle());

    //SmartDashboard.putNumber("Drive PID Target Position", Robot.drive.getTargetPosition());
    //SmartDashboard.putNumber("Drive kP", Robot.drive.getDriveKP());
    //SmartDashboard.putNumber("Drive kI", Robot.drive.getDriveKI());
    //SmartDashboard.putNumber("Drive kD", Robot.drive.getDriveKD());
    //SmartDashboard.putNumber("Drive kF", Robot.drive.getDriveKF());

  }

  @Override
  protected void initDefaultCommand() {
    //Keep this empty
  }

  public void updateVisionPIDValues(){
     Robot.drive.setVisionTargetPosition(SmartDashboard.getNumber("Turn PID Target Position", 0));
     System.out.println(Robot.drive.getVisionTargetPosition());
     Robot.drive.setVisionKP(SmartDashboard.getNumber("Turn kP", 0));
     System.out.println(Robot.drive.getVisionKP());
     Robot.drive.setVisionKI(SmartDashboard.getNumber("Turn kI", 0));
     Robot.drive.setVisionKD(SmartDashboard.getNumber("Turn kD", 0));
     Robot.drive.setVisionKF(SmartDashboard.getNumber("Turn kF", 0));
  }

  public void updateDrivePIDValues(){
    Robot.drive.setTargetPosition(SmartDashboard.getNumber("Drive PID Target Position", 0));
    System.out.println(Robot.drive.getTargetPosition());
    Robot.drive.setDriveKP(SmartDashboard.getNumber("Drive kP", 0));
    System.out.println(Robot.drive.getDriveKP());
    Robot.drive.setDriveKI(SmartDashboard.getNumber("Drive kI", 0));
    Robot.drive.setDriveKD(SmartDashboard.getNumber("Drive kD", 0));
    Robot.drive.setDriveKF(SmartDashboard.getNumber("Drive kF", 0));
 }
  

}
