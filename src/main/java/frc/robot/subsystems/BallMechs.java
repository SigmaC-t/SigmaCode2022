package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants;

public class BallMechs extends SubsystemBase{
    //Initialization of Intake Motors
    private static CANSparkMax intakeMotorF = new CANSparkMax(Constants.INTAKE_MOTOR_FRONT, MotorType.kBrushed);
    private static CANSparkMax intakeMotorB = new CANSparkMax(Constants.INTAKE_MOTOR_BACK, MotorType.kBrushed);

    private static CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushless);
    private static CANSparkMax shooterMotorTwo = new CANSparkMax(Constants.SHOOTER_MOTOR_TWO, MotorType.kBrushless);

    //Initialization of Hopper Motors
    private static CANSparkMax hopperMotor = new CANSparkMax(Constants.HOPPER_MOTOR_ONE, MotorType.kBrushed);
    private static CANSparkMax hopperMotor2 = new CANSparkMax(Constants.HOPPER_MOTOR_TWO, MotorType.kBrushed);

    //Initialization of Cylinders
     public DoubleSolenoid ArmBringerUpperF = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
     public DoubleSolenoid ArmBringerUpperFo = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);
     public DoubleSolenoid ArmBringerUpperB = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 5);
     public DoubleSolenoid ArmBringerUpperBa = new DoubleSolenoid(PneumaticsModuleType.CTREPCM , 6, 7);


    //public Ultrasonic ballSensor_hopper = new Ultrasonic(3, 4);
    //public AnalogPotentiometer BallSensor = new AnalogPotentiometer(1);

    //Initialization of IR Sensors
    //Digital Input class used to get a simple boolean value from the IR sensors.
    //returns false if it sees something, returns true otherwise. 
    public DigitalInput sensorBot = new DigitalInput(0);
    public DigitalInput sensorMid = new DigitalInput(1);
    public DigitalInput sensorTop = new DigitalInput(2);
    //public Counter counterMid = new Counter(Counter.Mode.kSemiperiod);
    //public Counter counterTop = new Counter(Counter.Mode.kSemiperiod);


    
    public void BallMech()
	{
		intakeMotorF.setIdleMode(IdleMode.kBrake);
        hopperMotor.setIdleMode(IdleMode.kBrake);
        //shooterMotor.setIdleMode(IdleMode.kBrake);
	}
    /*public void shooter(double speed){
        shooterMotor.set(speed);
        RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    } */
    public void intake (double speed, boolean extend){
           
            extendIntake(extend, ArmBringerUpperF);
            extendIntake(extend, ArmBringerUpperFo);

            if (RobotContainer.m_rightBumper.get()){

                System.out.println(ArmBringerUpperF.get());
                System.out.println(ArmBringerUpperFo.get());
                //BallSensor.get();
                System.out.println(sensorBot.get());


            }

            intakeMotorF.set(speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);

    }

    public void intakeTwo (double speed, boolean extend){

        extendIntake(extend, ArmBringerUpperB);
        extendIntake(extend, ArmBringerUpperBa);

        if (RobotContainer.m_leftBumper.get()){
            System.out.println(ArmBringerUpperB.get());
            System.out.println(sensorBot.get());

            intakeMotorB.set(speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);

        }
    }

       public void extendIntake(boolean extend, DoubleSolenoid intake){
           if (extend){

            intake.set(Value.kReverse);
           // System.out.println("It's extended");

           } else {
               
            intake.set(Value.kForward);
            //System.out.println("It's not extended");

           }
       }

      //  }
    
     /* public void outake(double speed /* boolean bumper*///)
	//{
       // if (bumper){
            //ArmBringerUpper.set(Value.kReverse);
            /* System.out.println(ArmBringerUpper.get());
            intakeMotor.set(-speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);

            */
      //  }
		
		/*else
		{
			intakeMotor.set(0.25);
		}
        */
  //  }

    public void shooter (double speed){
         shooterMotor.set(speed);
         shooterMotorTwo.set(-speed);
        
    }
    public void hopper(double speed){
       if (sensorBot.get()){
            //System.out.println(ballSensor_hopper.getEchoChannel());
            //System.out.println(ballSensor_hopper.getRangeMM());
            //System.out.println(ballSensor_hopper.getRangeInches());
            //System.out.println("Ball is detected");
            //System.out.println(ballSensor_hopper.isEnabled());
           // System.out.println(BallSensor.get());
            hopperMotor.set(0);
      } else {
          hopperMotor.set(speed);
          //System.out.println("The intake is intaking" + ": " + BallSensor.get());
        }
    }


    //Make intake run independetly from able to run intake independently from hopper.


    //Determines the position of the balls and acts accordingly
    // [0, 0 ,0] = No balls, intake as normal.
    // [1, 0, 0] = One ball in intake area, move upwards
    // [0, 1, 0] = One ball in the middle, stop hopper (allow intake as normal?)
    // [1, 1, 0] = One ball in intake area, one ball in middle, move upwards
    // [0, 0, 1] = One ball at top, stop intake, allow for shooting.
    int counter = 0; 
    int ballState = 0;
   public void Balls(){
    //int ballState = 0;
    switch (ballState){

        case 0:
        System.out.println("First Stage");
        if (!sensorBot.get()){
            hopperMotor.set(.35);
            ballState = 1;
            
        }

        break;

        case 1:
        //!sensorBot means ball is present. Inverts the DigitalInput output and turns false to true. 
        if (!sensorMid.get())
        {
            System.out.println("Second stage");
            hopperMotor.set(0);
            ballState = 2;
        }
        break;

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
    }
    
}
