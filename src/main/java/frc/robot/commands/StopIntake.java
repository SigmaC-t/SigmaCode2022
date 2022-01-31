// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.BallMechs;

public class StopIntake extends CommandBase {
  /** Creates a new StopIntake. */
  public StopIntake() {
    addRequirements(RobotContainer.m_BallMechs);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute( ) { 
    double speed = 0;
    RobotContainer.m_BallMechs.intake(speed);
    RobotContainer.m_BallMechs.shooter(speed);
    RobotContainer.m_BallMechs.hopper(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
