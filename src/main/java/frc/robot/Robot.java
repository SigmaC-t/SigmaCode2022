// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BallMech;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.Drivestraight;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.BallMechs;
import com.revrobotics.RelativeEncoder;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public PneumaticHub hub = new PneumaticHub();
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  


  private RobotContainer m_robotContainer;
  /** 
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //RobotContainer.m_BallMechs.counterBot.setUpSource(0);
    //RobotContainer.m_BallMechs.counterBot.setSemiPeriodMode(true);
    //hub.enableCompressorDigital();
    hub.enableCompressorAnalog(90, 120); // This is where it was 120 PSI. was 105
    RobotContainer.m_drivetrain.gearShifter.set(Value.kReverse); //kReverse is low gear
   // RobotContainer.m_BallMechs.ArmBringerUpperB.set(Value.kReverse);
   // RobotContainer.m_BallMechs.ArmBringerUpperF.set(Value.kReverse);
    

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    RobotContainer.m_SigmaSight.updateValues();
    //RobotContainer.m_BallMechs.BallSensor.get();
    CommandScheduler.getInstance().run();
    //RobotContainer.navX.updateAHRS();
    //RobotContainer.navX.update();
    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand =  RobotContainer.m_drivetrain.getAutonomousCommand(); // BasicAuto();
   // RobotContainer.navX.resetAngle();

    // schedule the autonomous command (exampl'e)
    if (m_autonomousCommand != null) {
      System.out.println("Autonomous Schedule");
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  int Counter = 0;
  @Override
  public void autonomousPeriodic() {

      /* RobotContainer.m_drivetrain.auto();

      if (Counter > 250){
        RobotContainer.m_drivetrain.tankDrive(0, 0);
      }
        */
  }

  @Override
  public void teleopInit() {
    RobotContainer.m_Hanger.lENC.setPosition(0);
    RobotContainer.m_Hanger.rENC.setPosition(0);

    
    RobotContainer.m_drivetrain.getRightEncoder().setPosition(0);
    RobotContainer.m_drivetrain.getLeftEncoder().setPosition(0);

   // RobotContainer.m_BallMechs.ArmBringerUpperPnuematic.set(Value.kReverse);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    //RobotContainer.navX.resetAngle();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  
    //System.out.println(hub.getPressure(0));
    // System.out.println("Left: " + RobotContainer.m_drivetrain.getLeftEncoder().getPosition());
    // System.out.println("Right " + RobotContainer.m_drivetrain.getRightEncoder().getPosition());

    SmartDashboard.putNumber("Pressure", hub.getPressure(0));



    
    //System.out.println(RobotContainer.m_BallMechs.sensorBot.get());
    //RobotContainer.m_SigmaSight.updateValues();
    //System.out.println(RobotContainer.m_BallMechs.sensorBot.get());
   // RobotContainer.m_BallMechs.intakeFront(0.5, true);
   // RobotContainer.m_BallMechs.intakeBack(0.5, true);
   //RobotContainer.m_BallMechs.shooterEncoder.getVelocity();
    }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  

  
}
