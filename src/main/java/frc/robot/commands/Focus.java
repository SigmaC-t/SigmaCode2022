// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SigmaSight;

public class Focus extends CommandBase {
  int counter;
  SigmaSight limelight = RobotContainer.m_SigmaSight;
  /** Creates a new Focus. */
  public Focus() {
    addRequirements(RobotContainer.m_drivetrain, RobotContainer.m_SigmaSight);
    counter = 0;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* RobotContainer.m_SigmaSight.Focus(RobotContainer.m_drivetrain);
    RobotContainer.m_SigmaSight.left_command = 0;
    RobotContainer.m_SigmaSight.right_command = 0;
    counter++;

    if (counter > 100){

      new runShooter(0.4);

    }

    */
    
   /* double heading_error = limelight.xVal;
    limelight.steering_adjust = limelight.turnKp * limelight.xVal;

    limelight.left_command += limelight.steering_adjust;
    limelight.right_command += limelight.steering_adjust;
    
    RobotContainer.m_drivetrain.tankDrive(limelight.left_command, limelight.right_command);
    System.out.println("Adjusting Aim");
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    RobotContainer.m_SigmaSight.aimAndRange(RobotContainer.m_drivetrain);
    //RobotContainer.m_SigmaSight.lineUpToShoot(RobotContainer.m_drivetrain);

    if (counter > 200){

      return true;

    }

    return false;
  }
}
