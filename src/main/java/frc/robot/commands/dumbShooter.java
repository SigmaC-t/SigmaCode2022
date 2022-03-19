// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class dumbShooter extends CommandBase {
  int counter = 0;
  int indexerCounter = 0;
  int shootAnyways = 0;
  double RPM = 4100;
  /** Creates a new dumbShooter. */
  public dumbShooter() {
    addRequirements(RobotContainer.m_BallMechs);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  //Count Balls that leave the shooter as an end condition

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    Timer time = new Timer();

    time.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter++;

    RPM = SmartDashboard.getNumber("RPM", 4100);

    RobotContainer.m_BallMechs.rpmShooter(RPM);

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

  if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) > 200){

    shootAnyways++;
    
  }

  if (indexerCounter > 20 && (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) < 200)){
    //Ball is there
    if (!RobotContainer.m_BallMechs.sensorTop.get()){
      RobotContainer.m_BallMechs.runHopper(0);
      System.out.println("Stopping the Hopper");
      Timer.delay(.3);


    //Ball is not there
    } else {

      RobotContainer.m_BallMechs.runHopper(1);
      System.out.println("No ball at top");

    }

  } else if (indexerCounter > 20 && shootAnyways > 50){

    RobotContainer.m_BallMechs.runHopper(1);
    System.out.println("Shooting Anyways");

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
    indexerCounter = 0;
    shootAnyways = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > 100){

      return true; 

    }

    return false;
  }
}
