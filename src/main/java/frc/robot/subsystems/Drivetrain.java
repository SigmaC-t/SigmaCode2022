package frc.robot.subsystems;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.lang.Math.*;
import java.lang.reflect.GenericDeclaration;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drivetrain extends SubsystemBase {

//Speed Controllers initial initialization.

public CANSparkMax leftMaster = null;
public CANSparkMax leftSlave = null;
public CANSparkMax leftSlave2 = null;
public CANSparkMax rightMaster = null;
public CANSparkMax rightSlave = null;
public CANSparkMax rightSlave2 = null;

private RelativeEncoder leftENC;
private RelativeEncoder rightENC;

private SlewRateLimiter leftLimiter;
private SlewRateLimiter rightLimiter;

private AHRS m_gyro;

public DoubleSolenoid gearShifter = null;

public DifferentialDriveOdometry m_odometry;

public double gearRatio = 9.91845616; //Low gear

//MotorControllerGroup initial initiablization
MotorControllerGroup left = null;
MotorControllerGroup right = null;

//DIfferential Drive initial initalization
DifferentialDrive differentialDrive = null;


//Encoder Values
//public RelativeEncoder leftTrainENC = leftSlave.getEncoder();
//public RelativeEncoder rightTrainENC = rightMaster.getEncoder();


private final double ENC_TICKS_PER_INCH = 0; //Calculate Encoder ticks per inch

public double rampRate = 0.1;


//Variables for NavX (Gyroscope)
double turnKP = 0.008;
	double distanceKP = 0.0000055;
	public int driveStraightState = 0;
	double desiredPosition = 0;
	double distance_adjust = 0;
	double currentAngle = 0;
  double averageEncoderPosition;
//Angle to turn at for NavX, takes the input angle compares it
// public boolean turnAngle(double angle, double a){
//   double speed = (angle - navX.angle) * 0.01;
//   tankDrive(-speed * a, speed * a);
//   if(Math.abs(angle - navX.angle) < 5){
//     tankDrive(0, 0);
//     System.out.print("done fuckeer");
//     return true;
//   }

//   return false;
// }
// public boolean driveStraight(double distance_inches, double speedScaleR, double speedScaleL){
//   switch(driveStraightState)
//   {
//     case 0: 
//     averageEncoderPosition = (rightTrainENC.getPosition() + leftTrainENC.getPosition())/ 2;
//   }

//   return false;
// }

  public Drivetrain() {

    //Instantiated Speed Controller objects (Assigned values Constants.DRIVETRAIN_X_X = CANID);
    CANSparkMax leftMaster = new CANSparkMax(Constants.DRIVETRAIN_LEFT_MASTER, MotorType.kBrushless);
    CANSparkMax leftSlave = new CANSparkMax(Constants.DRIVETRAIN_LEFT_SLAVE, MotorType.kBrushless);
    CANSparkMax leftSlave2 = new CANSparkMax(Constants.DRIVETRAIN_LEFT_SLAVE2, MotorType.kBrushless);
    CANSparkMax rightMaster = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_MASTER, MotorType.kBrushless);
    CANSparkMax rightSlave = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_SLAVE, MotorType.kBrushless);
    CANSparkMax rightSlave2 = new CANSparkMax(Constants.DRIVETRAIN_RIGHT_SLAVE2, MotorType.kBrushless);


    leftLimiter = new SlewRateLimiter(1);
    rightLimiter = new SlewRateLimiter(1);

    leftMaster.restoreFactoryDefaults();
    leftSlave.restoreFactoryDefaults();
    leftSlave2.restoreFactoryDefaults();
    rightMaster.restoreFactoryDefaults();
    rightSlave.restoreFactoryDefaults();
    rightSlave2.restoreFactoryDefaults();

    leftENC = leftMaster.getEncoder();
    rightENC = rightMaster.getEncoder();

    m_gyro = new AHRS(SPI.Port.kMXP);

    //Grouped the Speed Controller objects.
    left = new MotorControllerGroup(leftMaster,  leftSlave,  leftSlave2);
    right = new MotorControllerGroup(rightMaster,  rightSlave,  rightSlave2); 
    
    gearShifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.GEAR_SHIFT, Constants.GEAR_SHIFT_TWO);

    //Inverted one side of drivetrain to drive forward same direction as other group. 
    left.setInverted(true);

    //Instantiated Differential Drive that manipulates the MotorGroups (Left and Right side of robot)
    differentialDrive = new DifferentialDrive(left, right);

    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    leftSlave2.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    rightSlave2.setIdleMode(IdleMode.kCoast);

    leftMaster.setSmartCurrentLimit(35);
    leftSlave.setSmartCurrentLimit(35);
    leftSlave2.setSmartCurrentLimit(35);
    rightMaster.setSmartCurrentLimit(35);
    rightSlave.setSmartCurrentLimit(35);
    rightSlave2.setSmartCurrentLimit(35);

    //leftMaster.setOpenLoopRampRate(0.1)

    leftMaster.burnFlash();
    leftSlave.burnFlash();
    leftSlave2.burnFlash();
    rightMaster.burnFlash();
    rightSlave.burnFlash();
    rightSlave2.burnFlash();

 

  }

  //tankDrive method used to allow take controller input to drive robot.
  //Used in DriveTank command.
  public void tankDrive (double left, double right){

  if (gearShifter.get() == Value.kForward){

    differentialDrive.tankDrive(-leftLimiter.calculate(left), -rightLimiter.calculate(right));
    System.out.println("Throttling High Gear");
     
   } else {
    differentialDrive.tankDrive(-left, -right);

    leftLimiter.calculate(left * 0.3);
    rightLimiter.calculate(right * 0.3);
   }
  }
    
    public void highGear(boolean gearState){
     if (gearState){

       gearShifter.set(Value.kForward);

    } else if (!gearState){

       gearShifter.set(Value.kReverse);

     }
 }

 public void arcadeDrive (double forward, double turn){

  if (gearShifter.get() == Value.kForward){

    differentialDrive.arcadeDrive(-leftLimiter.calculate(forward), -rightLimiter.calculate(turn));
    System.out.println("Throttling High Gear");
     
   } else {

    differentialDrive.arcadeDrive(-forward, -turn);

    leftLimiter.calculate(forward * 0.3);
    rightLimiter.calculate(turn * 0.3);

   }

 }

  
 @Override
 public void periodic() {
   m_odometry.update(
   m_gyro.getRotation2d(), nativeUnitsToDistanceMeters(leftENC.getPosition()), nativeUnitsToDistanceMeters(rightENC.getPosition()));

  // SmartDashboard.putNumber("Left Motor 1", leftMaster.getOutputCurrent());
  // SmartDashboard.putNumber("Left Motor 2", leftSlave.getOutputCurrent());
  // SmartDashboard.putNumber("Left Motor 3", leftSlave2.getOutputCurrent());

   // This method will be called once per scheduler run
 }
 
 public Pose2d getPose(){

   return m_odometry.getPoseMeters();

 }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
   return new DifferentialDriveWheelSpeeds(velocityToMetersPerSeconds(leftENC.getVelocity()), velocityToMetersPerSeconds(rightENC.getVelocity()));

 }

 public void resetOdometry(Pose2d pose){
   resetEncoders();
   m_odometry.resetPosition(pose, m_gyro.getRotation2d());

 }

 public void resetEncoders(){
   leftENC.setPosition(0);
   rightENC.setPosition(0);

 }

 public void tankDriveVolts (double leftVolts, double rightVolts){
   left.setVoltage(leftVolts);
   right.setVoltage(rightVolts);
   differentialDrive.feed();
 }

 public double getAverageEncoderDistance(){
   return ((nativeUnitsToDistanceMeters(leftENC.getPosition()) + nativeUnitsToDistanceMeters(rightENC.getPosition())) / 2.0);
 }

 public RelativeEncoder getLeftEncoder(){
   return leftENC;
 }

 public RelativeEncoder getRightEncoder(){
   return rightENC;

 }

 public void setMaxOutpet(double maxOutput){
   differentialDrive.setMaxOutput(maxOutput);
 }

 public void zeroHeading(){
   m_gyro.reset();
 }
 public double getTurnRate(){
   return m_gyro.getRate();
 }

 public double nativeUnitsToDistanceMeters(double sensorCounts){
  double motorRotations = (double)sensorCounts;
  double wheelRotations = motorRotations / (gearRatio);
  double positionMeters = wheelRotations * 0.319024;

  return positionMeters;

 }

 public double velocityToMetersPerSeconds (double RPM){

  double wheelRotations = RPM / gearRatio;
  double positionMetersPerMinute = wheelRotations * 0.319024;
  double positionMetersPerSecond = positionMetersPerMinute / 60;

  return positionMetersPerSecond;


 }

 public Command getAutonomousCommand (){

  var autoVoltageConstraint =
  new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
    Constants.kDriveKinematics,
    10
 );

 TrajectoryConfig config = new TrajectoryConfig(Constants.kMaxSpeedMetersPerSecond, Constants.kMaxAccelerationMetersPerSecondSquared).setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint);

Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)), List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);

RamseteCommand ramseteCommand = new RamseteCommand(
  exampleTrajectory,
  RobotContainer.m_drivetrain::getPose,
      new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.ksVolts,
                                 Constants.kvVoltSecondsPerMeter,
                                 Constants.kaVoltSecondsSquaredPerMeter),
      Constants.kDriveKinematics,
      RobotContainer.m_drivetrain::getWheelSpeeds,
      new PIDController(Constants.kPDriveVel, 0, 0),
      new PIDController(Constants.kPDriveVel, 0, 0),
      RobotContainer.m_drivetrain::tankDriveVolts,
      RobotContainer.m_drivetrain);

      return ramseteCommand.andThen(() -> RobotContainer.m_drivetrain.tankDriveVolts(0, 0));
      
 }
  
  //Counter used to track the time. 
  //Incrementation of counter happens in 20 ms segments (default refresh rate of execute())
  //Counter * 20 / 1000 = seconds passed.
  int counter = 0;

  //MVP autonomous code that moves the robot backwards for 5 seconds
  public void auto(double speed) {

    if (counter < 250) {

     tankDrive(speed, speed); // Was 0.75;

    }

    counter++;
  }






  // Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a {@link
 * edu.wpi.first.math.trajectory.TrapezoidProfile} instead.
 */
public class SlewRateLimiter {
  private final double m_rateLimit;
  private double m_prevVal;
  private double m_prevTime;

  /**
   * Creates a new SlewRateLimiter with the given rate limit and initial value.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   * @param initialValue The initial value of the input.
   */
  public SlewRateLimiter(double rateLimit, double initialValue) {
    m_rateLimit = rateLimit;
    m_prevVal = initialValue;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }

  /**
   * Creates a new SlewRateLimiter with the given rate limit and an initial value of zero.
   *
   * @param rateLimit The rate-of-change limit, in units per second.
   */
  public SlewRateLimiter(double rateLimit) {
    this(rateLimit, 0);
  }

  /**
   * Filters the input to limit its slew rate.
   *
   * @param input The input value whose slew rate is to be limited.
   * @return The filtered value, which will not change faster than the slew rate.
   */
  public double calculate(double input) {
    double currentTime = WPIUtilJNI.now() * 1e-6;
    double elapsedTime = currentTime - m_prevTime;
    if (input > m_prevVal && m_prevVal < 0 || input < m_prevVal && m_prevVal > 0){

      m_prevVal +=
      MathUtil.clamp(input - m_prevVal, -m_rateLimit * elapsedTime, m_rateLimit * elapsedTime);

    } else {

      m_prevVal = input;

    }
    
    m_prevTime = currentTime;
    return m_prevVal;
  }

  /**
   * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
   *
   * @param value The value to reset to.
   */
  public void reset(double value) {
    m_prevVal = value;
    m_prevTime = WPIUtilJNI.now() * 1e-6;
  }
}



}

