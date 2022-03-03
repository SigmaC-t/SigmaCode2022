// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class runIntakeF extends CommandBase {
  private double speed;
  private double hopper;
  /** Creates a new runIntake. */
  public runIntakeF(double x, double y) {
    addRequirements(RobotContainer.m_BallMechs);
    speed = x;
    hopper = y;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //double speed = .5;
    RobotContainer.m_BallMechs.intakeFront(speed, hopper, true);
    System.out.println("Intake is working");

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_BallMechs.intakeFront(0, hopper, false);
   // RobotContainer.m_BallMechs.intakeFront(0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
