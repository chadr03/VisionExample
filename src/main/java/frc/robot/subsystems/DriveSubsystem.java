/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.robot.commands.ManualDriveCommand;


/**
 * Add your docs here.
 */
public class DriveSubsystem extends Subsystem {
  //Control Global Variables
  
  //Dive Distance PID Constants
  private double targetPosition = 0; //Feet
  private double driveKP = 0;
  private double driveKD = 0;
  private double driveKI = 0;
  private double driveKF = 0;

  private double lastTimestamp = 0;
  private double errorSum = 0;
  private double lastError = 0;
  private double iZone =  1.0;
  

  //Turn with Vision PID Constants
  private double visionTargetPosition = 0; //Feet
  private double visionKP = 0;
  private double visionKD = 0;
  private double visionKI = 0;
  private double visionKF = 0;

  private double visionLastTimestamp = 0;
  private double visionErrorSum = 0;
  private double visionLastError = 0;
  private double visionIZone =  5.0;

  //Drive Sraight Gyro
  private double gyroKP = 0.07;
  
  //Set up motor controllers
  Spark left1 = new Spark(RobotMap.left1Port);
  Spark left2 = new Spark(RobotMap.left2Port);
  Spark right1 = new Spark(RobotMap.right1Port);
  Spark right2 = new Spark(RobotMap.right2Port);
  
  //Groups motor controllers that will always have the same value
  SpeedControllerGroup leftMaster = new SpeedControllerGroup(left1, left2);
  SpeedControllerGroup rightMaster = new SpeedControllerGroup(right1, right2);

  //Sets up differental drive
  DifferentialDrive drive = new DifferentialDrive(leftMaster, rightMaster);

  //Set up sensors
  Encoder rightEncoder = new Encoder(RobotMap.rightEncoderA, RobotMap.rightEncoderB, true, Encoder.EncodingType.k4X);
  Encoder leftEncoder = new Encoder(RobotMap.leftEncoderA, RobotMap.leftEncoderB, false, Encoder.EncodingType.k4X);
  public ADXRS450_Gyro gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);
  PowerDistributionPanel pdp = new PowerDistributionPanel();

  public DriveSubsystem(){
    rightEncoder.setDistancePerPulse(RobotMap.DRIVE_ENCODER_PULSE_PER_FT);
    leftEncoder.setDistancePerPulse(RobotMap.DRIVE_ENCODER_PULSE_PER_FT);
    gyro.calibrate();

  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new ManualDriveCommand());
  }



  //Set up methods to control drive subsystem and to get values from sensors
  public void teleopDrive(double move, double turn) {
    drive.arcadeDrive(move, turn);
  }

  public void tankDrive(double leftSpeed, double rightSpeed){
    drive.tankDrive(leftSpeed, rightSpeed);
  }

  public void driveStraight(double move){
    double error = gyro.getAngle();
    double turn = error * gyroKP;
    turn = Math.copySign(turn, move);
    teleopDrive(move, turn);

  }

  public int rightEncoderCount(){
    return rightEncoder.get();
  }

  public double rightEncoderSpeed(){
    return rightEncoder.getRate();
  }

  public double rightEncoderDistance(){
    return rightEncoder.getDistance();
  }

  public double rightEncoderInch(){
    return rightEncoderDistance() * 12;
  }

  public int leftEncoderCount(){
    return leftEncoder.get();
  }

  public double leftEncoderSpeed(){
    return leftEncoder.getRate();
  }

  public double leftEncoderDistance(){
    return leftEncoder.getDistance();
  }

  public double leftEncoderInch(){
    return leftEncoderDistance() * 12;
  }

  public double inchesTraveled(){
    return 12* (rightEncoder.getDistance() + leftEncoder.getDistance()) / (2); //average of both encoder distance and converts from ft to inch
  }

  public void zeroEncoders(){
    rightEncoder.reset();
    leftEncoder.reset();
  }

  public void zeroSensors(){
    zeroEncoders();
    gyro.reset();
  }

  public double getAngle(){
    return gyro.getAngle();
  }



  //PID Command for Drive to distance with encoder
  public void drivePID(){
    double error = targetPosition - rightEncoder.getDistance();
    double dt = Timer.getFPGATimestamp() - lastTimestamp;
    driveKF = Math.copySign(driveKF, error);

    if(Math.abs(error)<iZone){
      errorSum = errorSum + error * dt;
    }else{
      errorSum = 0;
    }

    double errorRate = (error - lastError)/dt;

    Double outputSpeed = driveKF + (driveKP * error) + (driveKI * errorSum) + (driveKD * errorRate);
    teleopDrive(outputSpeed, 0);

    SmartDashboard.putNumber("Output", outputSpeed);

    lastTimestamp = Timer.getFPGATimestamp();
    lastError = error;
    
    
  }

  public void initDrivePID(){
    zeroEncoders();
    errorSum = 0;
    lastTimestamp = Timer.getFPGATimestamp();
  }



  //Getters and Setters for Drive PID

  public void setTargetPosition(double position){
    targetPosition = position;
  }
  public double getTargetPosition(){
    return targetPosition;
  }

  public void setDriveKP(double kP){
    driveKP = kP;
  }
  public double getDriveKP(){
    return driveKP;
  }

  public void setDriveKI(double kI){
    driveKI = kI;
  }
  public double getDriveKI(){
    return driveKI;
  }

  public void setDriveKD(double kD){
    driveKD = kD;
  }
  public double getDriveKD(){
    return driveKD;
  }

  public void setDriveKF(double kF){
    driveKF = kF;
  }
  public double getDriveKF(){
    return driveKF;
  }



  //PID Command for turn with vision to distance with encoder
  public void visionTurnPID(){
    double error = visionTargetPosition + Robot.vision.getYaw();
    double dt = Timer.getFPGATimestamp() - visionLastTimestamp;
    visionKF = Math.copySign(visionKF, error);

    if(Math.abs(error)<visionIZone){
      visionErrorSum = visionErrorSum + error * dt;
    }else{
      visionErrorSum = 0;
    }

    double errorRate = (error - visionLastError)/dt;

    Double outputTurn = visionKF + (visionKP * error) + (visionKI * visionErrorSum) + (visionKD * errorRate);
    teleopDrive(0, outputTurn);

    SmartDashboard.putNumber("Vision Turn Value", outputTurn);

    visionLastTimestamp = Timer.getFPGATimestamp();
    visionLastError = error;
    
    
  }

  public void initVisionTurnPID(){
    visionErrorSum = 0;
    visionLastTimestamp = Timer.getFPGATimestamp();
  }


  //Getters and Setters for Drive PID

  public void setVisionTargetPosition(double position){
    visionTargetPosition = position;
  }
  public double getVisionTargetPosition(){
    return visionTargetPosition;
  }

  public void setVisionKP(double kP){
    visionKP = kP;
  }
  public double getVisionKP(){
    return visionKP;
  }

  public void setVisionKI(double kI){
    visionKI = kI;
  }
  public double getVisionKI(){
    return visionKI;
  }

  public void setVisionKD(double kD){
    visionKD = kD;
  }
  public double getVisionKD(){
    return visionKD;
  }

  public void setVisionKF(double kF){
    visionKF = kF;
    System.out.println(kF);
  }
  public double getVisionKF(){
    return visionKF;
  }

}
