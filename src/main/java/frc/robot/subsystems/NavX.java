// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX extends SubsystemBase {
  // Creates a new navX. 
  public AHRS AHRS;
  public double yaw, angle, pitch, roll;

  public NavX() {
    AHRS gyro = new AHRS(SPI.Port.kMXP);
  }
  public void updateAHRS()
    {
        yaw = AHRS.getYaw();
        angle = AHRS.getAngle();
        pitch = AHRS.getPitch();
        roll = AHRS.getRoll();
    }

    public void resetAngle()
    {
        AHRS.zeroYaw();
    }

    public void update()
    {
        SmartDashboard.putNumber("angle", angle);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  
}
