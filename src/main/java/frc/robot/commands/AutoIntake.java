// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoIntake extends CommandBase {

  int counter;
  /** Creates a new AutoIntake. */
  public AutoIntake() {
    addRequirements(RobotContainer.m_BallMechs);
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

    RobotContainer.m_BallMechs.intakeBack(0.8, 0.8, 0.4, true);
    System.out.println("Intake is working");
    counter++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    RobotContainer.m_BallMechs.intakeBack(0, 0, 0, false);
    RobotContainer.m_BallMechs.ArmBringerUpperB.set(Value.kForward); 



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > 200){

      return true;

    }
    return false;
  }
}
