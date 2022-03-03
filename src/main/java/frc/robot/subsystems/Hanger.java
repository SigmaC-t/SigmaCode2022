// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hanger extends SubsystemBase {
  public CANSparkMax hangR = new CANSparkMax(Constants.HANGER_MOTOR, MotorType.kBrushless);
  public CANSparkMax hangL= new CANSparkMax(Constants.HANGER_MOTOR_TWO, MotorType.kBrushless);

  public static DoubleSolenoid hangARMS = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARMS_FWD, Constants.ARMS_BACK);
  

  SparkMaxPIDController m_PidControllerR = hangR.getPIDController();
  SparkMaxPIDController m_PidControllerL = hangL.getPIDController();

  RelativeEncoder rENC = hangR.getEncoder();
  RelativeEncoder lENC = hangL.getEncoder();

  

  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, speed;
  //public static DoubleSolenoid hangLARM = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.LEFT_ARM_FWD, Constants.LEFT_ARM_BACK);
  //Test individual motors to set direction 
  //find out if encoders are absolute (if power is lost, do encoders remember);
  //get all pneumatic sworking
  //add logictoslowdownhopper
  //addlogictostopspinninghopper 
  /** Creates a new Hanger. */

  public Hanger() {

    hangARMS.set(Value.kReverse);
    kP = 0.1;
    kI = 0.0001;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_PidControllerL.setOutputRange(kMinOutput, kMaxOutput);
    m_PidControllerR.setOutputRange(kMinOutput, kMaxOutput);

    speed = 0.1;
 
  }

  public boolean lowerArms(){

    hangARMS.set(Value.kForward);

    return true;  

  }

  public boolean upArms(){

    hangARMS.set(Value.kReverse);

    return true;

  }

  public void ascendArms(){

   // if (hangARMS.get() == Value.kForward){


      hangL.set(speed);
      hangR.set(-speed);
      System.out.println("HangL: " + lENC.getPosition());
      System.out.println("HangR: " + rENC.getPosition());

   // }

    

  }

  public void descendArms(){

    hangR.set(speed);
    hangL.set(-speed);
    System.out.println("HangL: " + lENC.getPosition());
    System.out.println("HangR: " + rENC.getPosition());
  }


  @Override
  public void periodic() {

   

   // m_PidControllerL.setReference(0, CANSparkMax.ControlType.kPosition);
  //  m_PidControllerR.setReference(0, CANSparkMax.ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
