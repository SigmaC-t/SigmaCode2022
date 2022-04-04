// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Drivestraight extends CommandBase {
  double speed;
  /** Creates a new Drivestraight. */
  public Drivestraight(double x) {
    addRequirements(RobotContainer.m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
    speed = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  int Counter = 0;

  @Override
  public void execute() {
    System.out.println("Please work");
    RobotContainer.m_drivetrain.tankDrive(speed, speed);
    Counter++;

    /*if (Counter > 50){
      RobotContainer.m_drivetrain.tankDrive(0, 0);
    }
    Counter++;
    */
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    Counter = 0;

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Counter > 20){
      
      System.out.println("DriveStraight is finished");
      return true;

    } else {

      return false;

    }
  }
}
