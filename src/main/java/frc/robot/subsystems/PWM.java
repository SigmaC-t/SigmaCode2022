// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import edu.wpi.first.wpilibj.TimedRobot;

// /** Add your docs here. */
// public class PWM {
//     public void AddressableLED(){
//         @Override
//         public void robotInit() {
//     // PWM port 9
//     // Must be a PWM header, not MXP or DIO
//         m_led = new AddressableLED(9);

//     // Reuse buffer
//     // Default to a length of 60, start empty output
//     // Length is expensive to set, so only set it once, then just update data
//         m_ledBuffer = new AddressableLEDBuffer(60);
//         m_led.setLength(m_ledBuffer.getLength());

//     // Set the data
//         m_led.setData(m_ledBuffer);
//         m_led.start();
//         }
//     }

// }
