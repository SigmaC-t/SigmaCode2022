// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
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
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.BallMech;
import frc.robot.commands.BasicAuto;
import frc.robot.commands.ComplexAuto;
import frc.robot.commands.ComplexAutoP2;
import frc.robot.commands.Drivestraight;
import frc.robot.commands.StopIntake;
import frc.robot.subsystems.BallMechs;

import java.io.IOException;
import java.nio.file.Path;

import javax.swing.plaf.basic.BasicTextUI;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private SendableChooser<Command> chooser = new SendableChooser<Command>();

  public PneumaticHub hub = new PneumaticHub();
  Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);
  String trajectoryJSON = "paths/works.wpilib.json";
  String trajectoryFIRST = "paths/firstd.wpilib.json";
  String trajectorySECOND = "paths/secondssss.wpilib.json";
  String trajectoryTHIRD = "paths/thirdooo.wpilib.json";
  String trajectoryFOURTH = "paths/fourthHAILMARY.wpilib.json"; //was fourthd

  Trajectory trajectory = new Trajectory();
  Trajectory first = new Trajectory();
  Trajectory second = new Trajectory();
  Trajectory third = new Trajectory();
  Trajectory fourth = new Trajectory();


  //DigitalInput magnet = new DigitalInput(6);


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
    //hub.enableCompressorDigital();
    hub.enableCompressorAnalog(90, 120); // This is where it was 120 PSI. was 105
    RobotContainer.m_drivetrain.gearShifter.set(Value.kReverse); //kReverse is low gear
    RobotContainer.m_SigmaSight.limelightTable.getEntry("stream").setNumber(0);

    chooser.setDefaultOption("2 Ball Auto", new BasicAuto());
    chooser.addOption("4 Ball Auto", new ComplexAuto(first, second, third, fourth));

    SmartDashboard.putData("Auto Chooser", chooser);



    
   try {

    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

   } catch (IOException ex){

    DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }

  first = RobotContainer.m_drivetrain.generateTrajectory(trajectoryFIRST);
  second = RobotContainer.m_drivetrain.generateTrajectory(trajectorySECOND);
  third = RobotContainer.m_drivetrain.generateTrajectory(trajectoryTHIRD);
  fourth = RobotContainer.m_drivetrain.generateTrajectory(trajectoryFOURTH);



  // SmartDashboard.putData("2 Ball Auto", new BasicAuto());
  // SmartDashboard.putData("4 Ball Auto", new ComplexAuto(first, second, third, fourth));



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
    CommandScheduler.getInstance().run();

    
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {

    //System.out.println(SmartDashboard.getString("Auto Command", chooser.getSelected().getName()));

  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
     
    RobotContainer.m_drivetrain.getRightEncoder().setPosition(0);
    RobotContainer.m_drivetrain.getLeftEncoder().setPosition(0);

    RobotContainer.m_drivetrain.resetOdometry(second.getInitialPose());

  m_autonomousCommand = new ComplexAuto(first, second, third, fourth);
   //m_autonomousCommand = new BasicAuto();
    // m_autonomousCommand = new ComplexAutoP2(third, fourth);
    // m_autonomousCommand = RobotContainer.m_drivetrain.getAutonomousCommand(fourth);//new BasicAuto();
    //m_autonomousCommand = chooser.getSelected();

    // schedule the autonomous command (example)
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

    // RobotContainer.m_drivetrain.leftMaster.setIdleMode(IdleMode.kCoast);
    // RobotContainer.m_drivetrain.leftSlave.setIdleMode(IdleMode.kCoast);
    // RobotContainer.m_drivetrain.leftSlave2.setIdleMode(IdleMode.kCoast);
    // RobotContainer.m_drivetrain.rightMaster.setIdleMode(IdleMode.kCoast);
    // RobotContainer.m_drivetrain.rightSlave.setIdleMode(IdleMode.kCoast);
    // RobotContainer.m_drivetrain.rightSlave2.setIdleMode(IdleMode.kCoast);

    RobotContainer.m_drivetrain.resetOdometry(new Pose2d(0, 0, new Rotation2d(0)));

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

    //System.out.println(RobotContainer.m_Hanger.magnet.get());
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
