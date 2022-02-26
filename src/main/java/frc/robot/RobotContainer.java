// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.*;
import frc.robot.commands.BallMech;
import frc.robot.commands.DriveTank;
import frc.robot.commands.DriveToRange;
import frc.robot.commands.Focus;
import frc.robot.subsystems.BallMechs;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SigmaSight;
import frc.robot.commands.StopIntake;
import frc.robot.commands.runHopper;
import frc.robot.commands.runIntakeB;
import frc.robot.commands.runIntakeF;
import frc.robot.commands.runShooter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.NavX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static Joystick driverController = new Joystick(Constants.DRIVER_CONTROLLER);
  public static final Drivetrain m_drivetrain = new Drivetrain();
  public static final BallMechs m_BallMechs = new BallMechs();
  public static final SigmaSight m_SigmaSight = new SigmaSight();
  public static JoystickButton m_buttonA, m_buttonB, m_buttonX, m_buttonXRaw, m_buttonY, m_leftBumper, m_leftBumperReleased, m_rightBumper, m_leftStick, m_rightStick, o_buttonBReleased;
  public static double m_leftTrigger, m_leftAnalogX, m_rightAnalogX, m_leftAnalogY, m_rightAnalogY;
  double m_rightTrigger;
  public static Button o_buttonA, o_buttonB, o_buttonX, o_buttonXReleased, o_buttonY, o_leftBumper, o_rightBumper, o_leftStick, o_rightStick;
  public static double o_leftTrigger, o_rightTrigger, o_leftAnalogX, o_rightAnalogX, o_leftAnalogY, o_rightAnalogY;
  public static XboxController mainController = new XboxController(0);
  public static XboxController operatorController = new XboxController(1);
  public static NavX navX = new NavX();
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    m_drivetrain.setDefaultCommand(new DriveTank());
    m_BallMechs.setDefaultCommand(new StopIntake());

    //Intake Code
    m_rightBumper.whenPressed(new runIntakeF());
    m_rightBumper.whenReleased(new StopIntake());

    m_leftBumper.whenPressed(new runIntakeB());
    m_leftBumper.whenReleased(new StopIntake());

    //Outake Code
   // m_leftBumper.whenPressed(new BallMech());
   // m_leftBumper.whenReleased(new StopIntake());

    //Hopper Code
    //m_buttonX.whenPressed(new runHopper());
    //m_buttonX.whenReleased(new StopIntake());

    m_buttonB.whenHeld(new Focus());

    //
    if (driverController.getRawAxis(2) > 0.5){

      new runIntakeB();

    }

    //m_buttonA.whenHeld(new DriveToRange());


    m_buttonX.whenPressed(new runShooter());
    m_buttonX.whenReleased(new StopIntake());
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
        m_buttonA = new JoystickButton(driverController, XboxController.Button.kA.value);
        //m_buttonB = new JoystickButton(driverController, 2);
        m_rightBumper = new JoystickButton(mainController, XboxController.Button.kRightBumper.value);
        m_leftBumper = new JoystickButton(mainController, XboxController.Button.kLeftBumper.value);
        m_buttonB = new JoystickButton(mainController, XboxController.Button.kB.value);
        m_buttonX = new JoystickButton(mainController, XboxController.Button.kX.value);
        m_leftTrigger = driverController.getRawAxis(2);
        m_rightTrigger = driverController.getRawAxis(3);
        /* 
        m_buttonX = mainController.getRawButtonPressed(3);
        m_buttonXRaw = mainController.getRawButton(3);
        m_buttonY = mainController.getRawButtonPressed(4);
        m_leftBumper = mainController.getRawButton(5);
        m_leftBumperReleased = mainController.getRawButtonReleased(5);
        m_rightBumper = mainController.getRawButton(6);
        m_leftStick = mainController.getRawButton(9);
        m_rightStick = mainController.getRawButton(10);
        m_leftTrigger = zeroValue(mainController.getRawAxis(2));
        m_rightTrigger = zeroValue(mainController.getRawAxis(3));
        m_leftAnalogX = zeroValue(mainController.getRawAxis(0));
        m_rightAnalogX = zeroValue(mainController.getRawAxis(4));
        m_leftAnalogY = zeroValue(mainController.getRawAxis(1)) * -1;
        m_rightAnalogY = zeroValue(mainController.getRawAxis(5)) * -1;

        o_buttonA = operatorController.getRawButton(1);
        o_buttonB = operatorController.getRawButton(2);
        o_buttonBReleased = operatorController.getRawButtonReleased(2);
        o_buttonX = operatorController.getRawButton(3);
        o_buttonY = operatorController.getRawButton(4);
        o_leftBumper = operatorController.getRawButton(5);
        o_rightBumper = operatorController.getRawButton(6);
        o_leftStick = operatorController.getRawButton(9);
        o_rightStick = operatorController.getRawButton(10);
        o_leftTrigger = zeroValue(operatorController.getRawAxis(2));
        o_rightTrigger = zeroValue(operatorController.getRawAxis(3));
        o_leftAnalogX = zeroValue(operatorController.getRawAxis(0));
        o_rightAnalogX = zeroValue(operatorController.getRawAxis(4));
        o_leftAnalogY = zeroValue(operatorController.getRawAxis(1)) * -1;
        o_rightAnalogY = zeroValue(operatorController.getRawAxis(5)) * -1;
   
  }
  private static double zeroValue(double realValue) 
  {
      if(realValue < 0.09 && realValue > -0.09)
      {
          return 0;
      }
      else
      {
          return 1;
      }
      */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
