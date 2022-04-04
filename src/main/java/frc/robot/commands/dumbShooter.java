// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class dumbShooter extends CommandBase {
  int counter = 0;
  int indexerCounter = 0;
  int shootAnyways = 0;
  int autoLimit;
  double RPM = 4100;
  double hoodedRPM = 5000;
  double error;
  /** Creates a new dumbShooter. */
  public dumbShooter(int limit) {
    addRequirements(RobotContainer.m_BallMechs);
    autoLimit = limit;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Count Balls that leave the shooter as an end condition

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    shootAnyways = 0;

    Timer time = new Timer();

    time.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter++;

    RPM = SmartDashboard.getNumber("RPM", 4100);
    hoodedRPM = SmartDashboard.getNumber("Hooded RPM", 4600);

    if (RobotContainer.m_BallMechs.hoodie.get() == Value.kForward){

      error = hoodedRPM;
      RobotContainer.m_BallMechs.rpmShooter(hoodedRPM);

    } else {

      RobotContainer.m_BallMechs.rpmShooter(RPM);
      error = RPM; 

    }

    //RobotContainer.m_BallMechs.rpmShooter(RPM);

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

    }

  if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - error) > 100){

    shootAnyways++;
    //System.out.println("Incrementing Shoot Anyways " + shootAnyways);
    
  }

  if (indexerCounter > 20 && (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - error) < 50)){
    //Ball is there
    if (!RobotContainer.m_BallMechs.sensorTop.get()){
      Timer.delay(0.1);
      RobotContainer.m_BallMechs.runHopper(0);
      System.out.println("Stopping the Hopper");
      RobotContainer.m_BallMechs.ballCount -= 1; //If shooter ceases to delete this
      System.out.println("Ball Count: " + RobotContainer.m_BallMechs.ballCount);
      Timer.delay(.3);


    //Ball is not there
    } else {

      RobotContainer.m_BallMechs.runHopper(.7);
      System.out.println("No ball at top");

    }

  // } else if (indexerCounter > 20 && shootAnyways > 100){

  //   RobotContainer.m_BallMechs.runHopper(1);
  //   System.out.println("Shooting Anyways");

   }



   }

 // 333 334 335
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    counter = 0;
    RobotContainer.m_BallMechs.shooter(0);
    RobotContainer.m_BallMechs.indexerMotor.set(0);
    RobotContainer.m_BallMechs.runHopper(0);
    RobotContainer.m_BallMechs.ballCount = 0; 
    indexerCounter = 0;
    shootAnyways = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > 200){

      return true; 

    } else if (DriverStation.isAutonomousEnabled()  && counter > autoLimit){ //If shooter ceases to work delete this

      return true;

    }

    return false;
  }
}
