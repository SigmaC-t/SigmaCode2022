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
    public static final double TURN_KP = -0.02;
    public static final double MIN_AIM_COMMAND = 0.2;//0.2;
    public static final double DISTANCE_KP = -0.1; // was -0.1

    //11, 4 = BACK
    //12, 3 = FRONT
    //13,2 = DRIVETRAIN
    //15, 0 = Hanger
    

    //Two Climber Motors

    //Ramsete Controller Constants
    //Need to change values below after System Identification tests
    public static final double ksVolts = 0.16977; //= //0.17239; //0.16848;
    public static final double kvVoltSecondsPerMeter = 3.8018; //3.7934; //Change the units according to the measurement of wheels
    public static final double kaVoltSecondsSquaredPerMeter = 0.21444; // 0.25027; //0.29465; 

    public static final double kPDriveVel = 3.6853;
    public static final double kTrackWidthMeter = 0.9613; //0.95016; //0.94409; // Properly characterize
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackWidthMeter);

    public static final double kMaxSpeedMetersPerSecond = 3; //0.77931;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    //Works well for most robots
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
    public static final int GEAR_SHIFT = 13;
    public static final int GEAR_SHIFT_TWO = 2;
    public static final int ARMS_FWD = 15;
    public static final int ARMS_BACK = 0;
    public static final int UP_MOTOR = 15;
    public static final int SECONDARY_ARMS_FWD = 0;
    public static final int SECONDARY_ARM_BACK = 0;



}