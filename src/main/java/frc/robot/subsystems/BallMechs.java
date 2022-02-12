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
    private static CANSparkMax intakeMotor = new CANSparkMax(Constants.INTAKE_MOTOR, MotorType.kBrushed);
    //private static CANSparkMax shooterMotor = new CANSparkMax(Constants.SHOOTER_MOTOR, MotorType.kBrushed);
    private static CANSparkMax hopperMotor = new CANSparkMax(Constants.HOPPER_MOTOR, MotorType.kBrushed);
     public DoubleSolenoid ArmBringerUpper = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    //public Ultrasonic ballSensor_hopper = new Ultrasonic(3, 4);
    public AnalogPotentiometer BallSensor = new AnalogPotentiometer(1);
    public Counter counterBot = new Counter(Counter.Mode.kSemiperiod);
    public Counter counterMid = new Counter(Counter.Mode.kSemiperiod);
    public Counter counterTop = new Counter(Counter.Mode.kSemiperiod);

    public void BallMech()
	{
		intakeMotor.setIdleMode(IdleMode.kBrake);
        hopperMotor.setIdleMode(IdleMode.kBrake);
        //shooterMotor.setIdleMode(IdleMode.kBrake);
	}
    /*public void shooter(double speed){
        shooterMotor.set(speed);
        RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    } */
    public void intake (double speed, boolean extend){
           
            extendIntake(extend);

            if (RobotContainer.m_rightBumper.get()){

                System.out.println(ArmBringerUpper.get());
                //BallSensor.get();
                System.out.println(counterBot.getPeriod());


            }

            intakeMotor.set(speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);

    }

       public void extendIntake(boolean extend){
           if (extend){

            ArmBringerUpper.set(Value.kReverse);
            System.out.println("It's extended");

           } else {
               
            ArmBringerUpper.set(Value.kForward);
            System.out.println("It's not extended");

           }
       }

      //  }
    
     public void outake(double speed /* boolean bumper */)
	{
       // if (bumper){
            //ArmBringerUpper.set(Value.kReverse);
            System.out.println(ArmBringerUpper.get());
            intakeMotor.set(-speed);
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      //  }
		
		/*else
		{
			intakeMotor.set(0.25);
		}
        */
    }

    public void shooter (double speed){
        // shooterMotor.set(speed);
        
    }
    public void hopper(double speed){
       if (BallSensor.get() > .8){
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


    public void Balls(){
    int ballState = 0;
    switch (ballState){

        case 0:
        hopperMotor.set(.35);
        ballState = 1;

        case 1:
        if (counterMid.getPeriod() > 0)
        {
            hopperMotor.set(0);
            ballState = 2;
        }

        case 2:
        if (counterBot.getPeriod() > 0 && counterMid.getPeriod() > 0){
            hopperMotor.set(.35);
            ballState = 3;
        }

        case 3: 
        if(counterTop.getPeriod() > 0){
            hopperMotor.set(0);
        }

        case 4: 
        hopperMotor.set(.5);
    }
    }
}
