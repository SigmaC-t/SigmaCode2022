// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class DriveToRange extends CommandBase {
  int counter;
  String length;
  /** Creates a new DriveToRange. */
  public DriveToRange(String x) {
    addRequirements(RobotContainer.m_drivetrain, RobotContainer.m_SigmaSight);
    length = x;
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
    RobotContainer.m_SigmaSight.getInRange(RobotContainer.m_drivetrain);
    RobotContainer.m_SigmaSight.left_command = 0;
    RobotContainer.m_SigmaSight.right_command = 0;
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > 20 && length.equals("short")){

      return true;
      
    } else if (Math.abs(RobotContainer.m_SigmaSight.yVal) < 2){

      return true;

    } else if (counter > 100){

      return true;

    }
    
    return false;
  }
}
