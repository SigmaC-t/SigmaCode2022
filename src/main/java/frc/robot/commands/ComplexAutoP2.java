// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ComplexAutoP2 extends SequentialCommandGroup {
  /** Creates a new ComplexAutoP2. */
  public ComplexAutoP2(Trajectory third, Trajectory fourth) {
    // Add your commands in the addCommands() call, e.g.                            
    addCommands(new InstantCommand(() -> { RobotContainer.m_drivetrain.resetOdometry(new Pose2d(7.621, 2.676, Rotation2d.fromDegrees(26)));}), new PathIntake(third, 2, 200),  new PathIntake(fourth, 0, 200), /* RobotContainer.m_drivetrain.getAutonomousCommand(fourth)*//*, new DriveToRange("short")*/ new runShooter(0.4)); //new ShooterSequenceDIST(5150)); //Attempting to shoot hig
  }
}
