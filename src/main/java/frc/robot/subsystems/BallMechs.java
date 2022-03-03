package frc.robot.subsystems;

import java.util.concurrent.DelayQueue;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

    public static CANSparkMax upMotor = new CANSparkMax(15, MotorType.kBrushed);

    private static CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless); //Change kBrushed back to kBrushless after testing
    private static CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);

    double HopperSpeed = 0.4;
    //toggle hopper 
    //11, 4 = BACK
    //12, 3 = FRONT
    //13,2 = DRIVETRAIN
    //15, 0 = Hanger

    //public static RelativeEncoder shooterEncoder = shooterMotor.getEncoder();
    //public static 
    public RelativeEncoder shooterEncoderTwo = shooterMotorTwo.getEncoder();
    
    public static PneumaticHub hub = new PneumaticHub();
    double pressure;

    //Initialization of Hopper Motors\[]

    public static CANSparkMax hopperMotor = new CANSparkMax(Constants.HOPPER_MOTOR_ONE, MotorType.kBrushed);
    // private static CANSparkMax hopperMotor2 = new CANSparkMax(Constants.HOPPER_MOTOR_TWO, MotorType.kBrushed);

    //Initialization of Cylinders 
     public DoubleSolenoid ArmBringerUpperF = new DoubleSolenoid(PneumaticsModuleType.REVPH, 11, 4); // 11, 4
     //public DoubleSolenoid ArmBringerUpperFr = new DoubleSolenoid(PneumaticsModuleType.REVPH, 2, 3);
     public DoubleSolenoid ArmBringerUpperB = new DoubleSolenoid(PneumaticsModuleType.REVPH, 12, 3); // 12, 3
     //public DoubleSolenoid ArmBringerUpperBa = new DoubleSolenoid(PneumaticsModuleType.REVPH , 6, 7);




    //public Ultrasonic ballSensor_hopper = new Ultrasonic(3, 4);
    //public AnalogPotentiometer BallSensor = new AnalogPotentiomete`r(1);

    //Initialization of IR Sensors
    //Digital Input class used to get a simple boolean value from the IR sensors.
    //returns false if it sees something, returns true otherwise. 
    public DigitalInput sensorBot = new DigitalInput(0);
    //public DigitalInput sensorTop = new DigitalInput(1);
    //public DigitalInput sensorTop = new DigitalInput(2);
    //public Counter counterMid = new Counter(Counter.Mode.kSemiperiod);
    //public Counter counterTop = new Counter(Counter.Mode.kSemiperiod);


    
    public void BallMech()
	{
		intakeMotorF.setIdleMode(IdleMode.kBrake);
        hopperMotor.setIdleMode(IdleMode.kBrake);
	}


    //********************Make intake into one function eventually

    public void intakeFront(double speed, double hopper, boolean extend){

       

            ArmBringerUpperF.set(Value.kForward);
            intakeMotorF.set(-speed);

      // if (sensorBot.get()){
            runHopper(hopper);
            upMotor.set(speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);

    //  } else if (!sensorBot.get()){

     //  runHopper(0);
     //   upMotor.set(0);

   //   } 


    }


    public void moveBallUp(){



    }
    
    public void intakeBack(double speed, double hopper, boolean extend){


        System.out.println(ArmBringerUpperB.get());

     //  if (sensorBot.get()){

            upMotor.set(speed);
            runHopper(hopper);


     //  } else if (!sensorBot.get()){

           // upMotor.set(0);
           // runHopper(0);
   
     //   }

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


  //Add code to integrate limelight with shooting. 
    public void shooter (double speed){
         shooterMotor.set(-speed);
         shooterMotorTwo.set(speed);
        
    }
    public void hopper(double speed){

          hopperMotor.set(speed);

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
      //  pressure = get
       // SmartDashboard.putNumber("Pressure: ", pressure);
    }
}
