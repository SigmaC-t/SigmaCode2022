// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorSensor extends SubsystemBase {
  private final ColorMatch m_colorMatcher;
  private final I2C.Port I2cPort;
  private final ColorSensorV3 m_colorSensor; 
  private final Color kBlue = new Color(0.143, 0.427, 0.429);
  private final Color kRed = new Color(0.561, 0.232, 0.114);
  

  /** Creates a new ColorSensor. */
  public ColorSensor() {

    m_colorMatcher = new ColorMatch();
    I2cPort = I2C.Port.kOnboard;
    m_colorSensor = new ColorSensorV3(I2cPort);

    m_colorMatcher.addColorMatch(kBlue);
    m_colorMatcher.addColorMatch(kRed);
    
    m_colorMatcher.setConfidenceThreshold(0.5);

    SmartDashboard.putString("Alliance Color: ", "Blue");
    


  }

  public String detectColor(){

    Color detectedColor = m_colorSensor.getColor();

    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == kBlue){

      System.out.println("A Blue Ball is Queued");
      return "Blue";
      

    }

    if (match.color == kRed){

      System.out.println("A Red Ball is Queued");
      return "Red";

    }
    
    System.out.println("The color is: " + detectedColor);
    return "No Color";

  }


  @Override
  public void periodic() {


    // This method will be called once per scheduler run
  }
}
