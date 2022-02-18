// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallMechs;


public class BallMech extends CommandBase {
  /** Creates a new BallMech. */
  public BallMech() {
    addRequirements(RobotContainer.m_BallMechs);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {



  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_BallMechs.Balls();
    System.out.println("Hopper is running");


    /*
    double speed = .8;
    if (RobotContainer.m_rightBumper.get()){
    RobotContainer.m_BallMechs.intake(speed, true);
    RobotContainer.m_BallMechs.hopper(speed);
    }
    else {
      //RobotContainer.m_BallMechs.outake(speed);
    }
    */
   // RobotContainer.m_BallMechs.outake(speed /*, RobotContainer.m_rightBumper*/);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_BallMechs.hopper(0);
    RobotContainer.m_BallMechs.intake(0, false);
   // RobotContainer.m_BallMechs.outake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
