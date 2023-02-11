// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private static final int NUM_LED_COUNT = 60;
  private final AddressableLED lightLED;
  private AddressableLEDBuffer bufferLED;
  /** Creates a new Lights. */
  public Lights() {
   lightLED = new AddressableLED(6);
   bufferLED = new AddressableLEDBuffer(NUM_LED_COUNT);
   lightLED.setLength(NUM_LED_COUNT);
  }
  public void dataSetter() {
    lightLED.setData(bufferLED);
    lightLED.start(); 
  }
  public void sameColorLED() {
    for(var ourlights = NUM_LED_COUNT; ourlights < NUM_LED_COUNT; ourlights++){
        bufferLED.setRGB(ourlights, 17, 17, 17);
    }
  }
  public void lightsOut() {
    for(var ourlights = NUM_LED_COUNT; ourlights < NUM_LED_COUNT; ourlights++){
        bufferLED.setRGB(ourlights, 0, 0,0); }
    
  }
  public void setLights(int red, int green, int blue) {
    bufferLED.setRGB(NUM_LED_COUNT, red, green, blue);
  }
  public void setCertainLights(int minLight, int maxLight, int red, int green, int blue) {
    for(int light = minLight; light < maxLight+1; light++) {
      bufferLED.setRGB(light, red, green, blue);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

}