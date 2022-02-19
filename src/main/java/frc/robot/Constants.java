// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int DRIVETRAIN_LEFT_MASTER = 4;
    public static final int DRIVETRAIN_LEFT_SLAVE = 5;
    public static final int DRIVETRAIN_LEFT_SLAVE2 = 6;
    public static final int DRIVETRAIN_RIGHT_MASTER = 1;
    public static final int DRIVETRAIN_RIGHT_SLAVE = 2;
    public static final int DRIVETRAIN_RIGHT_SLAVE2 = 3;

    //MISC motors
    public static final int INTAKE_MOTOR_FRONT = 7;
    public static final int INTAKE_MOTOR_BACK = 8;;
    // public static final int SHOOTER_MOTOR = 9;
    public static final int HOPPER_MOTOR_ONE = 11;
    public static final int HOPPER_MOTOR_TWO = 12;


    //Controller Inputs
    public static final int DRIVER_CONTROLLER = 0;
    public static final int DRIVER_CONTROLLER_MOVE_AXIS = 1;
    public static final int DRIVER_CONTROLLER_ROTATE_AXIS = 5;

    //Limelight Constants
    public static final double TURN_KP = -0.03;
    public static final double MIN_AIM_COMMAND = -.1;
    public static final double DISTANCE_KP = -0.1;


    public static final int SHOOTER_MOTOR = 9;
    public static final int SHOOTER_MOTOR_TWO = 10;
    

}
