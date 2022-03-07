// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoShooter extends CommandBase {
  int counter;
  /** Creates a new AutoShooter. */
  public AutoShooter() {
    addRequirements(RobotContainer.m_BallMechs);
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


    RobotContainer.m_BallMechs.shooter(-0.8);
    RobotContainer.m_BallMechs.indexerMotor.set(-0.7);
    
    if (RobotContainer.m_BallMechs.shooterEncoderTwo.getVelocity() >= -4350){
      counter++;

      if (counter > 30){
       System.out.println(RobotContainer.m_BallMechs.shooterEncoderTwo.getVelocity());
       RobotContainer.m_BallMechs.hopperMotor.set(-0.7);

      }
      

     }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > 100){

      return true;

    }
    return false;
  }
}

