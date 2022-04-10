// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.subsystems.BallMechs;
import frc.robot.RobotContainer;

public class runShooter extends CommandBase {
  double speed;
  int counter;
  /** Creates a new runShooter. */
  public runShooter(double x) {
    addRequirements(RobotContainer.m_BallMechs);
    speed = x;
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    counter = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //System.out.println("Running Shooter");9
    //\
    //speed .8 is ideal for shooting in the top hub. 
    /*

    //Make sure all balls are out by detecting top sensor changing values. 
    //Once top sensor has changed value X amount of times, stop motors

    /*if (RobotContainer.m_BallMechs.Balls()){
      System.out.println("Shooting"); */
      //double speed = .8;
      //Create a function that waits for the shooter to get up to speed before moving the hopper and indexer.
    RobotContainer.m_BallMechs.shooter(-speed);
    RobotContainer.m_BallMechs.indexerMotor.set(-speed);
    counter++;

        if (counter > 20){

        RobotContainer.m_BallMechs.hopperMotor.set(-0.7);

        }

       
      
       
   
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_BallMechs.shooter(0);
    RobotContainer.m_BallMechs.hopperMotor.set(0);
    RobotContainer.m_BallMechs.indexerMotor.set(0);
    counter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
