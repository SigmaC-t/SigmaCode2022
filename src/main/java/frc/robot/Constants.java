// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    //Drivetrain Motors
    public static final int DRIVETRAIN_LEFT_MASTER = 1;
    public static final int DRIVETRAIN_LEFT_SLAVE = 2;
    public static final int DRIVETRAIN_LEFT_SLAVE2 = 3;
    public static final int DRIVETRAIN_RIGHT_MASTER = 4;
    public static final int DRIVETRAIN_RIGHT_SLAVE = 5;
    public static final int DRIVETRAIN_RIGHT_SLAVE2 = 6;

    //Ball Mechanism Motors (Intake & Hopper)
    public static final int INTAKE_MOTOR_FRONT = 7; 
    public static final int INTAKE_MOTOR_BACK = 8; 
    public static final int HOPPER_MOTOR_ONE = 11;
    //public static final int HOPPER_MOTOR_TWO = 12;

    //Shooter Motors
    public static final int SHOOTER_MOTOR = 9;
    public static final int SHOOTER_MOTOR_TWO = 10;
    public static final int INDEXER_MOTOR = 12;

    //Climber Motors 
    public static final int HANGER_MOTOR = 13;
    public static final int HANGER_MOTOR_TWO = 14;


    //Controller Inputs
    public static final int DRIVER_CONTROLLER = 0;
    public static final int OP_CONTROLLER = 1;
    public static final int DRIVER_CONTROLLER_MOVE_AXIS = 1;
    public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 5;

    //Limelight Constants09
    public static final double TURN_KP = -0.03;
    public static final double MIN_AIM_COMMAND = 0.2;
    public static final double DISTANCE_KP = -0.05; // was -0.1

    //11, 4 = BACK
    //12, 3 = FRONT
    //13,2 = DRIVETRAIN
    //15, 0 = Hanger
    

    //Two Climber Motors

    //Ramsete Controller Constants
    //Need to change values below after System Identification tests
    public static final double ksVolts = 0.27737;
    public static final double kvVoltSecondsPerMeter = 3.6926; //Change the units according to the measurement of wheels
    public static final double kaVoltSecondsSquaredPerMeter = 0.42827; 

    public static final double kPDriveVel = 4.6575;
    public static final double kTrackWidthMeter = .8; // Properly characterize
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeter);

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Works well for most robots
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int GEAR_SHIFT = 13;
    public static final int GEAR_SHIFT_TWO = 2;
    public static final int ARMS_FWD = 15;
    public static final int ARMS_BACK = 0;
    public static final int UP_MOTOR = 15;



}