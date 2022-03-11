package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import java.lang.Math.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxRelativeEncoder.Type;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsControlModule;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Drivetrain extends SubsystemBase {
//navX initialization
NavX navX = new NavX();

//Speed Controllers initial initialization.

public CANSparkMax leftMaster = null;
public CANSparkMax leftSlave = null;
public CANSparkMax leftSlave2 = null;
public CANSparkMax rightMaster = null;
public CANSparkMax rightSlave = null;
public CANSparkMax rightSlave2 = null;

public DoubleSolenoid gearShifter = null;

public DifferentialDriveOdometry m_odometry;


//MotorControllerGroup initial initiablization
MotorControllerGroup left = null;
MotorControllerGroup right = null;

//DIfferential Drive initial initalization
DifferentialDrive differentialDrive = null;

//Encoder Values
//public RelativeEncoder leftTrainENC = leftSlave.getEncoder();
//public RelativeEncoder rightTrainENC = rightMaster.getEncoder();


private final double ENC_TICKS_PER_INCH = 0; //Calculate Encoder ticks per inch




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
    //Grouped the Speed Controller objects.
    left = new MotorControllerGroup(leftMaster,  leftSlave,  leftSlave2);
    right = new MotorControllerGroup(rightMaster,  rightSlave,  rightSlave2); 
    
    gearShifter = new DoubleSolenoid(PneumaticsModuleType.REVPH, Constants.GEAR_SHIFT, Constants.GEAR_SHIFT_TWO);

    //Inverted one side of drivetrain to drive forward same direction as other group. 
    left.setInverted(true);

    //Instantiated Differential Drive that manipulates the MotorGroups (Left and Right side of robot)
    differentialDrive = new DifferentialDrive(left, right);

    leftMaster.setIdleMode(IdleMode.kCoast);
    leftSlave.setIdleMode(IdleMode.kCoast);
    leftSlave2.setIdleMode(IdleMode.kCoast);
    rightMaster.setIdleMode(IdleMode.kCoast);
    rightSlave.setIdleMode(IdleMode.kCoast);
    rightSlave2.setIdleMode(IdleMode.kCoast);

  }

  //tankDrive method used to allow take controller input to drive robot.
  //Used in DriveTank command.
  public void tankDrive (double left, double right){

   differentialDrive.tankDrive(-left, -right);
  
  }

    public void highGear(boolean gearState){
     if (gearState){

       gearShifter.set(Value.kReverse);

    } else if (!gearState){

       gearShifter.set(Value.kForward);

     }
 }

  

  @Override
  public void periodic() {
    
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
}

