// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.lang.Math;

public class Hanger extends SubsystemBase {
  public CANSparkMax hangR = new CANSparkMax(Constants.HANGER_MOTOR, MotorType.kBrushless);
  public CANSparkMax hangL= new CANSparkMax(Constants.HANGER_MOTOR_TWO, MotorType.kBrushless);

  public Encoder leftArmENC;
  public Encoder rightArmENC; 

  public DutyCycleEncoder rightArmENCO;
  public DutyCycleEncoder leftArmENCO;

  public static DoubleSolenoid hangARMS = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.ARMS_FWD, Constants.ARMS_BACK);
  //public static DoubleSolenoid secondaryARMS = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.SECONDARY_ARMS_FWD, Constants.SECONDARY_ARM_BACK);
  

  SparkMaxPIDController m_PidControllerR = hangR.getPIDController();
  SparkMaxPIDController m_PidControllerL = hangL.getPIDController();
  public RelativeEncoder rENC = hangR.getEncoder();
  public RelativeEncoder lENC = hangL.getEncoder();

  PIDController leftArmPID;
  PIDController rightArmPID;

  public DigitalInput magnet = new DigitalInput(6);

  double maxPosition = 128; //95
  double minPosition = 0;
  //double midPosition = 50;

  //100 rENC, -100 lENC. from max to min

  DigitalInput hangSensor = new DigitalInput(1);
  double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, speed;
  //public static DoubleSolenoid hangLARM = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.LEFT_ARM_FWD, Constants.LEFT_ARM_BACK);
  //Test individual motors to set direction 
  //find out if encoders are absolute (if power is lost, do encoders remember);
  //get all pneumatic sworking
  //add logictoslowdownhopper
  //addlogictostopspinninghopper 
  /** Creates a new Hanger. */

  public Hanger() {

    // rightArmENC = new Encoder(0, 1);
    // leftArmENC = new Encoder(2, 3);
    

    hangR.setIdleMode(IdleMode.kBrake);
    hangL.setIdleMode(IdleMode.kBrake);

    //hangARMS.set(Value.kReverse);
    hangARMS.set(Value.kReverse); //kForward is arms back
    kP = 0.1;
    kI = 0.0001;
    kD = 1;
    kIz = 0;
    kFF = 0;
    kMaxOutput = 1;
    kMinOutput = -1;

    m_PidControllerL.setOutputRange(kMinOutput, kMaxOutput);
    m_PidControllerR.setOutputRange(kMinOutput, kMaxOutput);

    speed = 0.5;

    lENC.setPosition(0);
    rENC.setPosition(0);

    // leftArmPID.setPID(0, 0, 0);
    // rightArmPID.setPID(0, 0, 0);
 
  }

  public boolean lowerArms(){

    hangARMS.set(Value.kForward);

    return true;  

  }

  public boolean upArms(){

    hangARMS.set(Value.kReverse);

    return true;

  }

  // public boolean secondaryArmsUP(){

  //   secondaryARMS.set(Value.kForward);

  //   return false;

  // }

  // public boolean secondaryArmsDOWN(){

  //   secondaryARMS.set(Value.kReverse);

  //   return true;

  // }

  public void ascendArms(double speed){

      hangR.set(-1);
      hangL.follow(hangR, true);
      System.out.println(rENC.getPosition());
      System.out.println(lENC.getPosition());

  }


   // if (hangARMS.get() == Value.kForward){

  // if (rENC.getPosition() < maxPosition){

  //     hangR.set(speed);
  //     hangL.set(-speed);
  //     System.out.println("rENC: " + rENC.getPosition());
  //  //   System.out.println("lENC " + lENC.getPosition());
      
  // } else if (rENC.getPosition() >= maxPosition){

  //   hangR.set(0);
  //   hangL.set(0);
  //   System.out.println("TOO HIGH");

 //}
 
     /* hangL.set(speed);
      hangR.set(-speed);
      System.out.println("HangL: " + lENC.getPosition());
      System.out.println("HangR: " + rENC.getPosition()); */

   // }

    

  //}

  public void descendHome (){

    hangR.set(0.2);
    hangL.set(-0.2);
    rENC.setPosition(0);

  }

  public void descendUnbound(){
    hangR.set(1);
    hangL.follow(hangR, true);
    
  }

  

  public void descendArms(double speed){

    if (rENC.getPosition() * -1 <= minPosition){

      hangR.set(0);
      hangL.follow(hangR, true);

    } else {

      hangR.set(1);
      hangL.follow(hangR, true);

    }
  }
// if (rENC.getPosition() > minPosition){

//       hangR.set(-speed);
//       hangL.set(speed);
//       System.out.println("rENC: " + rENC.getPosition());
//     //  System.out.println("lENC " + lENC.getPosition());

//  } else if (rENC.getPosition() <= minPosition){

//      hangR.set(0);
//      hangL.set(0);
//      System.out.println("TOO LOW");
// //     rENC.setPosition(0);

  // }
   
   /* hangR.set(speed);
    hangL.set(-speed);
    System.out.println("HangL: " + lENC.getPosition());
    System.out.println("HangR: " + rENC.getPosition()); */

 // }


 // public void 
  int climbStage = 0;
  // public void autoHanger(){
  
  //   switch (climbStage){
  //     case 0:
  //     if(rENC.getPosition() < maxPosition){
  //     hangL.set(0.1);
  //     hangR.set(-0.1);
  //     }
  //     else if (rENC.getPosition() >= maxPosition){
  //       climbStage = 1;
  //     }
      
  //     case 1: 
  //     if(rENC.getPosition() > minPosition){
  //       hangL.set(-0.1);
  //       hangR.set(0.1);
  //     } else if(rENC.getPosition() <= minPosition){
  //       climbStage = 2;
  //     }

  //     case 2:
  //     if(rENC.getPosition() < midPosition){
  //       hangL.set(0.1);
  //       hangR.set(-0.1);
  //     }
  //     else if(rENC.getPosition() >= midPosition){
  //       climbStage = 3;
  //     }
  //     case 3:
  //     if(hangARMS.get() == Value.kReverse){
  //       hangARMS.set(Value.kForward);
  //       //delay
  //       climbStage = 4;
  //     }
      

  //     case 4: 
  //     if(rENC.getPosition() < maxPosition){
  //       hangL.set(0.1);
  //       hangR.set(-0.1);
  //     }
  //     else if (rENC.getPosition() >= maxPosition){
  //       climbStage = 5;
  //     }
      
  //     case 5:
  //     //if(rENC.getPosition() )
  //   }
  // }

    //Add a encoder position requirement to pneumatically deploying arm. 


  int hangState = 0;
  int counter = 0;
  public void autoClimb(){

    switch (hangState){

      case 0: 
      counter++;
      if(counter < 50){

        descendArms(0.3);

      }
      else{
        counter = 0;
        hangState = 1;
      }

      break;

      case 1: 
      if (rENC.getPosition() > 5){

        descendArms(1);

      } else {

        hangState = 2;

      }

      break;

      case 2: 

      if (rENC.getPosition() <= 30){

        ascendArms(.3);

      } else {

        hangState = 3;

      }

      break;

      case 3: 
      counter++;
      lowerArms();
      if (counter > 10){

        hangState = 4;
        counter = 0;

      }

      break;

      case 4:
      if (rENC.getPosition() < maxPosition){

        ascendArms(0.5);

      } else {

        upArms();
        hangState = 5;

      }

      break;

      case 5: 
      if (rENC.getPosition() > 5){

        descendArms(1);

      }

      break;

    }
  }

  //Work on PID loop.
  //if sensor is added, use it here.
  // public boolean resetToStart(){

  //   hangL.set(-0.1);
  //   hangR.set(0.1);
  // }

  //   if (rENC.getPosition() == 0 && lENC.getPosition()== 0){

  //     hangL.set(-0.1);
  //     hangR.set(0.1);

  //     if (rENC.getPosition() > minPosition){

  //       hangL.set(-0.1);
  //       hangR.set(0.1);

  //     }


  //   }

    


  // }
  

  @Override
  public void periodic() {

   

   // m_PidControllerL.setReference(0, CANSparkMax.ControlType.kPosition);
  //  m_PidControllerR.setReference(0, CANSparkMax.ControlType.kPosition);
    // This method will be called once per scheduler run
  }
}
