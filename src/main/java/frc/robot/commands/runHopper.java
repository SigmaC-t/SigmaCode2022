// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class runHopper extends CommandBase {
  private double speed;
  private int counter;
  
  /** Creates a new runHopper. */
  public runHopper(double x) {
   // addRequirements(RobotContainer.m_BallMechs);
    speed = x; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 30;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
  RobotContainer.m_BallMechs.runHopper(speed);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_BallMechs.runHopper(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (counter > 10){

    //   return true;

    // }

    return false;
  }
}
