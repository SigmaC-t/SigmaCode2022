// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class Outtake extends CommandBase {
  /** Creates a new Outtake. */
  public Outtake() {
    addRequirements(RobotContainer.m_BallMechs);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.m_BallMechs.intakeFront(-0.9, -0.8, -0.4, true);
    RobotContainer.m_BallMechs.intakeBack(-0.9, -0.8, -0.4, true);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_BallMechs.intakeFront(0, 0, 0, false);
    RobotContainer.m_BallMechs.intakeBack(0, 0, 0,  false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
