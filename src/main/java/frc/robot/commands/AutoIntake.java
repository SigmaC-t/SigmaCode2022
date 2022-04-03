// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class AutoIntake extends CommandBase {

  int counter;
  int flag;
  int climit;
  /** Creates a new AutoIntake. */
  public AutoIntake(int stage, int limit) {
    flag = stage;
    climit = limit;
    addRequirements(RobotContainer.m_BallMechs);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    RobotContainer.m_BallMechs.ballCount = 0;
    counter = 0;

    Timer time = new Timer();

    time.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    RobotContainer.m_BallMechs.intakeBack(1, 1, 0.4, true);
    System.out.println("Intake is working");
    counter++;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    counter = 0;
    RobotContainer.m_BallMechs.intakeBack(0, 0, 0, false);
    RobotContainer.m_BallMechs.ArmBringerUpperB.set(Value.kForward); 



  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (counter > climit){

      return true;

    } else if (RobotContainer.m_BallMechs.ballCount == 1 && flag == 1){

      Timer.delay(0.5);

      return true;

    } else if (RobotContainer.m_BallMechs.ballCount == 2 && flag == 2){

      Timer.delay(0.5);
      return true;
       
    }

    return false;
  }
}
