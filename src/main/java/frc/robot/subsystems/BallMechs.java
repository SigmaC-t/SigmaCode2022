package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.CAN;
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
    //public DoubleSolenoid ArmBringerUpperPnuematic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    //public Ultrasonic ballSensor_hopper = new Ultrasonic(3, 4);
    //double teamp = ballSensor_hopper.getRangeInches();
    public AnalogPotentiometer BallSensor = new AnalogPotentiometer(0);

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
    public void intake(double speed /*, boolean bumper */){
       // if (bumper){
            // ArmBringerUpperPnuematic.set(Value.kForward);
           // System.out.println(ArmBringerUpperPnuematic.get());
            intakeMotor.set(speed);
            //System.out.println("Motor is going");
            RobotContainer.mainController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
      //  }
			
    }
    
     public void outake(double speed /* boolean bumper */)
	{
       // if (bumper){
           // ArmBringerUpperPnuematic.set(Value.kReverse);
            //System.out.println(ArmBringerUpperPnuematic.get());
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
    
}
