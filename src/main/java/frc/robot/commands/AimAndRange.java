// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SigmaSight;

public class AimAndRange extends CommandBase {

  SigmaSight limelight = RobotContainer.m_SigmaSight;
  /** Creates a new AimAndRange. */
  public AimAndRange() {
    addRequirements(RobotContainer.m_SigmaSight, RobotContainer.m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

      double heading_error = -limelight.xVal;
      double distance_error = -limelight.yVal;
      double steering_adjust = 0;
      if (limelight.xVal > 1.0){
        steering_adjust = limelight.turnKp * heading_error - limelight.minAimCommand;
  
      } else if (limelight.xVal < 1.0){
  
        steering_adjust = limelight.turnKp * heading_error + limelight.minAimCommand;
      }
  
      double distance_adjust = limelight.distanceKP * distance_error;

      limelight.left_command -= steering_adjust + distance_adjust;
      limelight.right_command +=  steering_adjust + distance_adjust;


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
