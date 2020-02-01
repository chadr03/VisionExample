/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class VisionSubsystem extends Subsystem {
  
  NetworkTable table;
  private double yaw; 
  private double pitch; 
  private double area; 
  private boolean is_valid;

  

  public VisionSubsystem(){
  table=NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("USB Camera-B4.09.24.1");


  yaw = table.getEntry("yaw").getDouble(0);
  pitch = table.getEntry("pitch").getDouble(0);
  area = table.getEntry("area").getDouble(0);
  is_valid = table.getEntry("is_valid").getBoolean(false);

  
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void periodic() {
    
    
    
    

    //System.out.println(yaw);
   

  }

  public double getYaw(){
    yaw = table.getEntry("yaw").getDouble(0);
    return yaw;
  }

  public double getPitch(){
    pitch = table.getEntry("pitch").getDouble(0);
    return pitch;
  }

  public double getArea(){
    area = table.getEntry("area").getDouble(0);
    return area;
  }

  public boolean isValid(){
    is_valid = table.getEntry("is_valid").getBoolean(false);
    return is_valid;
  }
  
}
