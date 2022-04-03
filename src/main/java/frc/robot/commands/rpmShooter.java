// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class rpmShooter extends CommandBase {
  double RPM;
  int counter = 0;
  int indexerCounter = 0;
  int shootAnyways = 0;
  int reset;
  int wait;
  double hopperSpeed = 0.8;
  boolean end;
  String correctColor = SmartDashboard.getString("Alliance Color: ", "Blue");
  String wrongColor = "Red";
  private int hopperState = 0;
  /** Creates a new delayedShooter. */
  public rpmShooter(int x) {
    addRequirements(RobotContainer.m_BallMechs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    indexerCounter = 0;
    RobotContainer.m_BallMechs.ballCount = 2;
    System.out.println("Base Counter: " + counter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RPM = SmartDashboard.getNumber("RPM", 4100);

  //Start spinning the flywheel.
      RobotContainer.m_BallMechs.rpmShooter(RPM);

    // if (RobotContainer.m_ColorSensor.detectColor().equals(wrongColor)){

    //   RobotContainer.m_BallMechs.rpmShooter(1000);

    // }


    //In order to seperate balls, experiment with indexer pulsing, instead of hopper, to add distance between balls

    //This first block of code runs the indexer and hopper backwards to seperate balls from flywheel
    //Allows for flywheel to get up to speed without accidentally ejecting a ball.
    indexerCounter++;
    if (indexerCounter < 20){

      RobotContainer.m_BallMechs.indexerMotor.set(1);

      if (RobotContainer.m_BallMechs.sensorBot.get()){

        RobotContainer.m_BallMechs.runHopper(-0.5);

      } else {

        RobotContainer.m_BallMechs.runHopper(0);
      }

    } else {

      RobotContainer.m_BallMechs.indexerMotor.set(-1);

    


    // Switch state controls the pulse length of the hopper. 
    //Also has main shooter code in it. 
    switch (hopperState){

      case 0: 

      // if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) > 200){
      //   System.out.println("Waiting for RPM: " + shootAnyways);
      //   shootAnyways++;

      // }

      if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) < 200){

        counter++;
        System.out.println("Incrementing Counter");

        if (RobotContainer.m_BallMechs.ballCount == 1){
          wait++;

          if (wait > 30){
          RobotContainer.m_BallMechs.runHopper(hopperSpeed);
          RobotContainer.m_BallMechs.ballCount = 0;
          }


        } else if (RobotContainer.m_BallMechs.ballCount == 0){

          if (counter > 100){

            counter = 0;
            RobotContainer.m_BallMechs.runHopper(0);
            end = true;
            

          }

        } else if (RobotContainer.m_BallMechs.ballCount == 2){

            if (counter < 10){

              RobotContainer.m_BallMechs.runHopper(hopperSpeed);
              System.out.println("Running Hopper. Counter = " + counter);

            } else if (counter >= 10) {

              hopperState = 1;
              RobotContainer.m_BallMechs.runHopper(0);
              System.out.println("Hopper needs to stop");

            }

            if (reset > 2){

              RobotContainer.m_BallMechs.ballCount = 1;
              System.out.println("Already shot 1 ball");
              
            }
      }

      } else if (shootAnyways > 50){

        System.out.println("Shooting Anyways");
        RobotContainer.m_BallMechs.runHopper(hopperSpeed);
        shootAnyways = 0;

      }

      break;

      case 1:

      counter -= 2;

      if (counter <= 0){

        hopperState = 0;
        reset++;
        System.out.println("Reset " + reset);


      }

      break;

    }
  }
  }
    
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_BallMechs.ballCount = 0;
    RobotContainer.m_BallMechs.ball = false;
    end = false;
    RobotContainer.m_BallMechs.shooter(0);
    RobotContainer.m_BallMechs.hopperMotor.set(0);
    RobotContainer.m_BallMechs.indexerMotor.set(0);
    indexerCounter = 0;
    reset = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (end){

      return true;

    }

    return false;
  }
}
