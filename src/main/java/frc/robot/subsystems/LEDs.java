// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
  AddressableLED led = new AddressableLED(6);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(3000);

  public LEDs() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void bootLed() {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      if (i % 2 == 0)
        ledBuffer.setRGB(i, 0, 0, 255); // Blue
      else
        ledBuffer.setRGB(i, 255, 247, 0); // Yellow
    }
  }

  public void holdNoteLed() {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 0, 255, 0);
    led.setData(ledBuffer);
  }

  public void noNotesLed() {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 255, 0, 0);
    led.setData(ledBuffer);
  }

  public void autonLed() {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 5, 237, 245);
    led.setData(ledBuffer);
  }

  public void sequenceLed() {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 255, 247, 0);
    led.setData(ledBuffer);
  }

  public void clear(){
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, 0, 0, 0);
    led.setData(ledBuffer);
  }

  public void start(){
    led.start();
  }

  public void stop(){
    led.stop();
  }
}