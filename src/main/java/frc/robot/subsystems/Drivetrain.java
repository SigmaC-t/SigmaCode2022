// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

//MotorControllerGroup initial initialization
MotorControllerGroup left = null;
MotorControllerGroup right = null;

//DIfferential Drive initial initalization
DifferentialDrive differentialDrive = null;

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


  /*public void DriveStraight(double distance_inches){

    averageEncoderPosition = m_rightEncoder.getPosition();




  }
*/
  //Counter used to track the time. 
  //Incrementation of counter happens in 20 ms
  //Counter * 20 / 1000 = seconds passed.
  int counter = 0;

  //Moves the robot backwards for 5 seconds. 
  public void auto() {

    if (counter < 250) {

     tankDrive(0.75, 0.75);

    }

    counter++;
  }
}

