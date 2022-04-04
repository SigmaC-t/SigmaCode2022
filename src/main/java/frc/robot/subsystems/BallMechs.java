package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.concurrent.TimeUnit;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class BallMechs extends SubsystemBase{
    //Initialization of Intake Motors
    public static CANSparkMax intakeMotorF = new CANSparkMax(Constants.INTAKE_MOTOR_FRONT, MotorType.kBrushed); 
    public static CANSparkMax intakeMotorB = new CANSparkMax(Constants.INTAKE_MOTOR_BACK, MotorType.kBrushed); 

    public static CANSparkMax indexerMotor = new CANSparkMax(Constants.INDEXER_MOTOR, MotorType.kBrushed);

    private SparkMaxPIDController shooterPID;
    public RelativeEncoder shooterENC;
    private static CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless); //Change kBrushed back to kBrushless after testing
    private static CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);

    private double kP, kI, kD, kIz, kMaxOutput, kMinOutput, maxRPM, kFF;

    public boolean ball;

    public int highRPM = 4100;

    public int ballCount;
    double HopperSpeed = 0.4;
    //toggle hopper 
    //11, 4 = BACK
    //12, 3 = FRONT
    //13,2 = DRIVETRAIN
    //15, 0 = Hanger

    //public static RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    //public static 
    public RelativeEncoder shooterEncoderTwo = shooterMotorTwo.getEncoder();

    //Initialization of Hopper Motors\[]

    public static CANSparkMax hopperMotor = new CANSparkMax(Constants.HOPPER_MOTOR_ONE, MotorType.kBrushed);
    // private static CANSparkMax hopperMotor2 = new CANSparkMax(Constants.HOPPER_MOTOR_TWO, MotorType.kBrushed);

    //Initialization of Cylinders 
     public DoubleSolenoid ArmBringerUpperF = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4); // 11, 4
     public DoubleSolenoid ArmBringerUpperB = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 3); // 12, 3
     public DoubleSolenoid hoodie = new DoubleSolenoid(PneumaticsModuleType.REVPH, 10, 5);

    //Initialization of IR Sensors
    //Digital Input class used to get a simple boolean value from the IR sensors.
    //returns false if it sees something, returns true otherwise. 
    public DigitalInput sensorBot = new DigitalInput(0);
   public DigitalInput sensorTop = new DigitalInput(3);
    //public Counter counterMid = new Counter(Counter.Mode.kSemiperiod);
    //public Counter counterTop = new Counter(Counter.Mode.kSemiperiod);


    
    public BallMechs()
	{
		intakeMotorF.setIdleMode(IdleMode.kCoast);
        intakeMotorB.setIdleMode(IdleMode.kCoast);
        hopperMotor.setIdleMode(IdleMode.kBrake);

        shooterPID = shooterMotorTwo.getPIDController();
        shooterENC = shooterMotorTwo.getEncoder();

        kP = 0.000060; //SmartDashboard.getNumber("P", 0.000060); //0.000000081036 * 2; // 0.000025000; //0.000000081036 * 2; //0.090966;
        kI = 0.00000045; //75; // SmartDashboard.getNumber("I", 0);  // 0.000000000000001;
        kD = 0;
        kIz = 350; //600;
        kFF = 0.000172; //SmartDashboard.getNumber("FF", 0.000172); //0.0044631 / 25.05; // new SimpleMotorFeedforward(0.1277, 0.1258, 0.0044631); // kS, kV, kA
        
        kMaxOutput = 1;
        kMinOutput = -1;
        maxRPM = 5700;
        
        hoodie.set(Value.kReverse);

        SmartDashboard.putNumber("P", kP);
        SmartDashboard.putNumber("I", kI);
        SmartDashboard.putNumber("D", kD);
        SmartDashboard.putNumber("FF", kFF);
        shooterPID.setP(kP);
        shooterPID.setI(kI);
        shooterPID.setD(kD);
        shooterPID.setIZone(kIz);
        shooterPID.setFF(kFF);
        shooterPID.setOutputRange(kMinOutput, kMaxOutput);

        shooterMotor.setIdleMode(IdleMode.kBrake);
        shooterMotorTwo.setIdleMode(IdleMode.kBrake);

        SmartDashboard.putNumber("RPM", 4100);
        SmartDashboard.putNumber("Hooded RPM", 5500);
        SmartDashboard.putNumber("Ball Count", ballCount);

	}


    //********************Make intake into one function eventually

    public void intakeFront(double speed, double hopper, double upSpeed, boolean extend){

       
            if (!sensorBot.get() && !ball){

                ballCount = 1;
                System.out.println("Ball Count: " + ballCount);

            } else if (ballCount == 1 && sensorBot.get()){

                System.out.println("Awaiting Second Ball");
                ball = true;
                
            } else if (ball && !sensorBot.get()){

                System.out.println("Ball Count: " + ballCount);
                ballCount = 2;
                
            }

            ArmBringerUpperF.set(Value.kForward);
            intakeMotorF.set(-speed);
            runHopper(hopper);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);



    }


    public void moveBallUp(){



    }
    
    public void intakeBack(double speed, double hopper, double upSpeed, boolean extend){

        if (!sensorBot.get() && !ball){

            ballCount = 1;
            System.out.println("Ball Count: " + ballCount);

        } else if (ballCount == 1 && sensorBot.get()){

            System.out.println("Awaiting Second Ball");
            ball = true;
            
        } else if (ball && !sensorBot.get()){

            System.out.println("Ball Count: " + ballCount);
            ballCount = 2;
            
        }


        System.out.println(ArmBringerUpperB.get());


            runHopper(hopper);

        ArmBringerUpperB.set(Value.kReverse);
        intakeMotorB.set(-speed);
        RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);


    }

       public void extendIntake(boolean extend, DoubleSolenoid intake){
           if (extend){

            intake.set(Value.kForward);
           // System.out.println("It's extended");

           } else  {
               
            intake.set(Value.kReverse);
            //System.out.println("It's not extended");

           }
       }


    public void rpmShooter(double RPM){

       // SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter);
        //System.out.println("FF: " );
        shooterPID.setReference(-RPM, ControlType.kVelocity);
        shooterMotor.follow(shooterMotorTwo, true);
        System.out.println("shooterENC: " + shooterENC.getVelocity());
        //shooterPID.setFF(feedForward.calculate((RPM/60), (RPM - shooterENC.getVelocity() / 60)) / RobotController.getBatteryVoltage());


    }

  //Add code to integrate limelight with shooting. 
    public void shooter (double speed){
         shooterMotor.set(-speed);
         shooterMotorTwo.set(speed);
        
    }
    public void hopper(double speed){

          hopperMotor.set(speed);

    }

    public boolean hoodieUp(){

        hoodie.set(Value.kReverse);
    
        return true;  
    
      }
    
      public boolean hoodieDown(){
    
        hoodie.set(Value.kForward);
    
        return true;
    
      }

    //Make intake run independetly from able to run intake independently from hopper.


//change of plans, when bottom sensor gets the bll, slowly move towards the top sensor then stop

    //Determines the position of the balls and acts accordingly
    // [0, 0 ,0] = No balls, intake as normal.
    // [1, 0, 0] = One ball in intake area, move upwards
    // [0, 1, 0] = One ball in the middle, stop hopper (allow intake as normal?)
    // [1, 1, 0] = One ball in intake area, one ball in middle, move upwards
    // [0, 0, 1] = One ball at top, stop intake, allow for shooting.
    int counter = 0; 
   public int ballState = 0;

    public void runHopper(double speed){
        hopperMotor.set(-speed);
    }

   public boolean Balls(){
    //int ballState = 0;
    switch (ballState){

        case 0:
        System.out.println("First Stage");
            hopperMotor.set(-0.5);
            ballState = 1;         
        break;

        case 1:
        //!sensorBot means ball is present. Inverts the DigitalInput output and turns false to true. 
        System.out.println("Second Stage");
        hopperMotor.set(-0.5);
        if (!sensorBot.get())
        {
            System.out.println("Ready to Shoot");
            hopperMotor.set(0);
            ballState = 2;
            return true;
        }
        break;

        //Testing code below, may not be necessary

        case 2: 
        System.out.println("Clean-up Stage");
        hopperMotor.set(1);
        //delay
        if (!sensorBot.get() == false && !sensorBot.get() == false){
            ballState = 0;
            //Ensure that all balls are out before going back to 0.
        }


        
    }


    return false;
        /*
        case 2:
        System.out.println("Third Stage");
        if (!sensorBot.get() && !sensorMid.get()){
            hopperMotor.set(.35);
            ballState = 3;
        }
        break;

        case 3: 
        System.out.println("Fourth Stage");
        if(!sensorTop.get()){
            hopperMotor.set(0);
        }
        break;

        case 4: 
        //This case may not be necesary. The actual shooting can be handled by another method specifically for
        System.out.println("Ready to Shoot");
        //hopperMotor.set(.5);
        break;
    }
    */
    }

    public void periodic(){

        shooterPID.setP(SmartDashboard.getNumber("P", kP));
        shooterPID.setI(SmartDashboard.getNumber("I", kI));
        shooterPID.setD(SmartDashboard.getNumber("D", kD));
        shooterPID.setFF(SmartDashboard.getNumber("FF", kFF));

      //  pressure = get
       // SmartDashboard.putNumber("Pressure: ", pressure);
    }
}
