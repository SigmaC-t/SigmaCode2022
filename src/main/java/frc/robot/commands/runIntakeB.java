// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class runIntakeB extends CommandBase {
  int counter;
  private double speed;
  private double hopper;
  /** Creates a new runIntakeB. */
  public runIntakeB(double x, double y) {
    addRequirements(RobotContainer.m_BallMechs);
    speed = x; 
    hopper = y;
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

    //double speed = .5;
    RobotContainer.m_BallMechs.intakeBack(speed, hopper, 0.4, true);
    
    System.out.println("Intake is working");
    counter++;


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_BallMechs.intakeBack(0, hopper, 0, false);
    //double speed = 0;
    //RobotContainer.m_BallMechs.intakeBack(speed, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    // if (counter > 200){

    //   RobotContainer.m_BallMechs.ArmBringerUpperB.set(Value.kForward);

    //   return true;

    // }



    return false;
  }
}
