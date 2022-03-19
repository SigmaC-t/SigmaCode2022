// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class dumbShooter extends CommandBase {
  int counter = 0;
  /** Creates a new dumbShooter. */
  public dumbShooter() {
    addRequirements(RobotContainer.m_BallMechs);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    counter++;
    RobotContainer.m_BallMechs.rpmShooter(4350);
    RobotContainer.m_BallMechs.indexerMotor.set(1);

    if (counter > 20){

    RobotContainer.m_BallMechs.runHopper(1);

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
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
