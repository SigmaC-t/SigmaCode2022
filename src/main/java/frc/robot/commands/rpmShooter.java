// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class rpmShooter extends CommandBase {
  int RPM;
  int counter = 0;
  int indexerCounter = 0;
  int shootAnyways = 0;
  private int hopperState = 0;
  /** Creates a new delayedShooter. */
  public rpmShooter(int x) {
    addRequirements(RobotContainer.m_BallMechs);
    RPM = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    indexerCounter = 0;
    System.out.println("Base Counter: " + counter);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.m_BallMechs.rpmShooter(RPM);
    indexerCounter++;
    if (indexerCounter < 20){

      RobotContainer.m_BallMechs.indexerMotor.set(1);

      if (RobotContainer.m_BallMechs.sensorBot.get()){

        RobotContainer.m_BallMechs.runHopper(-0.1);

      } else {

        RobotContainer.m_BallMechs.runHopper(0);
      }

    } else {

      RobotContainer.m_BallMechs.indexerMotor.set(-1);

    }



    switch (hopperState){

      case 0: 

      if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) > 200){

        shootAnyways++;

      }

      if (Math.abs((RobotContainer.m_BallMechs.shooterENC.getVelocity() * -1) - RPM) < 200){

        counter++;
        System.out.println("Incrementing Counter");

        if (counter < 10){

          RobotContainer.m_BallMechs.runHopper(0.8);
          System.out.println("Running Hopper. Counter = " + counter);

        } else if (counter >= 10) {

          hopperState = 1;
          RobotContainer.m_BallMechs.runHopper(0);
          System.out.println("Hopper needs to stop");

        }

      } else if (shootAnyways > 50){

        System.out.println("Shooting Anyways");
        RobotContainer.m_BallMechs.runHopper(0.8);
        shootAnyways = 0;

      }

      break;

      case 1:

      counter -= 2;

      if (counter <= 0){

        hopperState = 0;

      }

      break;

    }


  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_BallMechs.shooter(0);
    RobotContainer.m_BallMechs.hopperMotor.set(0);
    RobotContainer.m_BallMechs.indexerMotor.set(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
