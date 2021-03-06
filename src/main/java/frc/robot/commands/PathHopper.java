// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathHopper extends ParallelDeadlineGroup {
  /** Creates a new PathHopper. */
  public PathHopper(Trajectory traj) {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(RobotContainer.m_drivetrain.getAutonomousCommand(traj));
    addCommands(new runHopper(1));
  }
}
