// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.RobotContainer;

public class DriveTank extends CommandBase {

  /** Creates a new DriveTank. */ 
  public DriveTank() {
    addRequirements(RobotContainer.m_drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double left = RobotContainer.driverController.getRawAxis(Constants.DRIVER_CONTROLLER_MOVE_AXIS);
    double right = RobotContainer.driverController.getRawAxis(4);//Constants.DRIVER_CONTROLLER_ROTATE_AXIS);
    RobotContainer.m_drivetrain.arcadeDrive(left, right); 
    //RobotContainer.m_drivetrain.tankDrive(left, RobotContainer.driverController.getRawAxis(5));

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_drivetrain.tankDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
