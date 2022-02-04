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

public class SigmaSight extends SubsystemBase {

  public boolean validTarget;
  public double xVal, yVal, area, skew;
  public double steering_adjust;
  public double distance_adjust;
  public double left_command;
  public double right_command;
  public double turnKp = Constants.TURN_KP;
  public double distanceKp, desiredArea;
  public double minAimCommand = Constants.MIN_AIM_COMMAND;

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tv = limelightTable.getEntry("tv");
  NetworkTableEntry tx = limelightTable.getEntry("tx");
  NetworkTableEntry ty = limelightTable.getEntry("ty");
  NetworkTableEntry ta = limelightTable.getEntry("ta");
  NetworkTableEntry ts = limelightTable.getEntry("ts");

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
   
  public boolean isValidTarget(){
    return ! (xVal == 0.0 && yVal == 0.0);

  }

  public void Focus(Drivetrain drivetrain){
    double heading_error = -xVal;
    steering_adjust = 0;

    if (xVal > 1.0){
      steering_adjust = turnKp * heading_error - minAimCommand;

    } else if (xVal < 1.0){

      steering_adjust = turnKp * heading_error + minAimCommand;
    }

    left_command -= steering_adjust;
    right_command += steering_adjust;
    
    drivetrain.tankDrive(left_command, right_command);
    System.out.println("Adjusting Aim");
    System.out.println("tx: " + xVal);
    System.out.println("Left Command: " + left_command);
    System.out.println("Right Command: " + right_command);
  }
  /** Creates a new SigmaSight. */
  public SigmaSight() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
