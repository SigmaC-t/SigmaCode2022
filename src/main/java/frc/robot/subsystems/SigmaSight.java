// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class SigmaSight extends SubsystemBase {

  public boolean validTarget;
  public double xVal, yVal, area, skew;
  public double steering_adjust;
  public double distance_adjust;
  public double left_command;
  public double right_command;
  public double turnKp = Constants.TURN_KP; //KPAIM
  public double distanceKP = Constants.DISTANCE_KP;
  public double desiredArea;
  public double minAimCommand = Constants.MIN_AIM_COMMAND;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = limelightTable.getEntry("tv");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  NetworkTableEntry ts = limelightTable.getEntry("ts");
  

  //Updates value and reflect updated values in SmartDashboard
  public void updateValues(){
    validTarget = isValidTarget();
    xVal = tx.getDouble(0.0);
    yVal = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    skew = ts.getDouble(0.0);

    SmartDashboard.putBoolean("tv", validTarget);
    SmartDashboard.putNumber("tx", xVal);
    SmartDashboard.putNumber("ty", yVal);
    SmartDashboard.putNumber("ta", area);
    SmartDashboard.putNumber("ts", skew);
  }


  //Returns boolean value if limelight sees a target.
  public boolean isValidTarget(){
    return ! (xVal == 0.0 && yVal == 0.0);

  }

  //Focus centers the crosshair of the Limelight to the middle of the vision target.
  public void Focus(Drivetrain drivetrain){
    double heading_error = -xVal;
    double steering_adjust = 0;
    if (xVal > 1.0){
      steering_adjust = turnKp * heading_error - minAimCommand;

    } else if (xVal < 1.0){

      steering_adjust = turnKp * heading_error + minAimCommand;
    }

    left_command -= steering_adjust;
    right_command +=  steering_adjust;


    drivetrain.tankDrive(left_command, right_command);
    System.out.println("Adjusting Aim");
    System.out.println("tx: " + xVal);
    System.out.println("ty: " + yVal);
    System.out.println("Left Command: " + left_command);
    System.out.println("Right Command: " + right_command);

  }
  

  public boolean lineUpToShoot(Drivetrain drivetrain)
    {
      int counter = 0;
        //double minAimCommand = 0.07;
        steering_adjust = turnKp * xVal;
        if(xVal > 0)
        {
            drivetrain.tankDrive(steering_adjust - minAimCommand, -steering_adjust + minAimCommand);
        }
        else if(xVal < 0)
        {
            drivetrain.tankDrive(steering_adjust + minAimCommand, -steering_adjust - minAimCommand);
        }
        if(Math.abs(xVal) < 2)
        {
            counter++;
        }
        return counter > 100;
    }

    public boolean aimAndRange(Drivetrain drivetrain)
    {
        if (xVal > 1.0)
        {
            steering_adjust = turnKp * -xVal - minAimCommand;
        }
        else if (xVal < 1.0)
        {
            steering_adjust = turnKp * -xVal + minAimCommand;
        }

        distance_adjust = distanceKP * (desiredArea - area);

        left_command = steering_adjust + distance_adjust * -1;
        right_command = -steering_adjust + distance_adjust * -1;

        System.out.println("leftSpeed: " + left_command);
        drivetrain.tankDrive(left_command, right_command);
        System.out.println("It is working");
        return (area > desiredArea - 0.5 && area < desiredArea + 0.5 && xVal > -1.2 && xVal < 1.2);
        
    }

  
  public void getInRange(Drivetrain drivetrain){
        double distance_error = -yVal;
        distance_adjust = distanceKP * distance_error; //(desiredArea - area);

        left_command += distance_adjust;
        right_command += distance_adjust;

        drivetrain.tankDrive(left_command, right_command);

        System.out.println("Driving to optimal distance");
        System.out.println("Distance error: " + distance_error);
        System.out.println("Distance adjust: " + distance_adjust);
        System.out.println("Left Command: " + left_command);
        System.out.println("Right CommandL " + right_command);
        
}
  
  /** Creates a new SigmaSight. */
  public SigmaSight() {}

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
