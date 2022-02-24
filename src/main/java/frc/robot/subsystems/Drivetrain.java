// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import java.lang.Math.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends SubsystemBase {

//Speed Controllers initial initialization.
public CANSparkMax leftMaster = null;
public CANSparkMax leftSlave = null;
public CANSparkMax leftSlave2 = null;
public CANSparkMax rightMaster = null;
public CANSparkMax rightSlave = null;
public CANSparkMax rightSlave2 = null;



//MotorControllerGroup initial initiablization
MotorControllerGroup left = null;
MotorControllerGroup right = null;

//DIfferential Drive initial initalization
DifferentialDrive differentialDrive = null;

private final double ENC_TICKS_PER_INCH = 0; //Calculate Encoder ticks per inch


//Variables for NavX (Gyroscope)
private int driveStraightState;
private double desiredPosition;
private double currentAngle;
private double distance_adjust;

/*private RelativeEncoder m_rightEncoder = rightMaster.getEncoder();
private RelativeEncoder m_leftEncoder = leftMaster.getEncoder();

double averageEncoderPosition; */



  public Drivetrain() {

    //Instantiated Speed Controller objects (Assigned values Constants.DRIVETRAIN_X_X = CANID);
    CANSparkMax leftMaster = new CANSparkMax(Constants.DRIVETRAIN_LEFT_MASTER, MotorType.kBrushless);
    CANSparkMax leftSlave = new CANSparkMax(Constants.DRIVETRAIN_LEFT_SLAVE, MotorType.kBrushless);
    CANSparkMax leftSlave2 = new CANSparkMax(Constants.DRIVETRAIN_LEFT_SLAVE2, MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_MASTER, MotorType.kBrushless);
    CANSparkMax rightSlave = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_SLAVE, MotorType.kBrushless);
    CANSparkMax rightSlave2 = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_SLAVE2, MotorType.kBrushless);

  

    //Grouped the Speed Controller objects.
    left = new MotorControllerGroup(leftMaster, leftSlave, leftSlave2);
    right = new MotorControllerGroup(rightMaster, rightSlave, rightSlave2);   

    //Inverted one side of drivetrain to drive forward same direction as other group. 
    left.setInverted(true);

    //Instantiated Differential Drive that manipulates the MotorGroups (Left and Right side of robot)
    differentialDrive = new DifferentialDrive(left, right);



  }

  //tankDrive method used to allow take controller input to drive robot.
  //Used in DriveTank command.
  public void tankDrive (double left, double right){

   differentialDrive.tankDrive(-left, -right);
  
  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  //Currently gets Neo 550 integrated encoders. When we get robot, change to SRX Mag Encoders using Alternative Encoder
  //Change to Encoder and install encoders into roborio. 
 /*RelativeEncoder m_leftEncoder = leftMaster.getEncoder();
 RelativeEncoder m_rightEncoder = rightMaster.getEncoder();

  
  //Autonomous method used to drive robot straight forward X amount of inches. Returns a boolean to tell when method ends. 
  public boolean DriveStraight(double distance_inches){

   double averageEncoderPositionL = m_leftEncoder.getPosition();
   double averageEncoderPositionR = m_rightEncoder.getPosition();


  switch (driveStraightState){

     case 0:
     averageEncoderPositionL = -m_leftEncoder.getPosition();
     desiredPosition = averageEncoderPositionL + (distance_inches * ENC_TICKS_PER_INCH); 
     driveStraightState = 1;
     currentAngle = RobotContainer.navX.angle;

      case 1: 
      double angle_adjust = (currentAngle - RobotContainer.navX.angle) * 0.008;
      distance_adjust = (desiredPosition - (averageEncoderPositionL) * Constants.DISTANCE_KP);
      tankDrive(-distance_adjust - angle_adjust, -distance_adjust + angle_adjust);
      if (Math.abs(distance_adjust) < 0.3){
        driveStraightState = 2;
      }
      break;

      case 2:
      tankDrive(0, 0);
      return true;

   }

  return false; 


  }

  //Autonomous method used to turn the robot X amount of deg. Returns a boolean to tell when method ends. 
  public boolean turnAngle(double angle){
    double speed = (angle - RobotContainer.navX.angle) * 0.01;
    tankDrive(-speed, speed);
    if (Math.abs(angle - RobotContainer.navX.angle) < 5){
      tankDrive(0, 0);
      System.out.println("Done Turning");
      return true;
    }

    return false;


  }
  
*/
  //Counter used to track the time. 
  //Incrementation of counter happens in 20 ms segments (default refresh rate of execute())
  //Counter * 20 / 1000 = seconds passed.
  int counter = 0;

  //MVP autonomous code that moves the robot backwards for 5 seconds
  public void auto() {

    if (counter < 250) {

     tankDrive(0.75, 0.75);

    }

    counter++;
  }
}

