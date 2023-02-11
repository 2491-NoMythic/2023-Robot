// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class SkiPlow extends SubsystemBase {
  /** Creates a new SkiPlow. */
  Solenoid skiPlowLeftPCM;
  Solenoid skiPlowDoublePCMLock;
  Solenoid skiPlowRightPCM;
  public SkiPlow() {
    skiPlowLeftPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    skiPlowRightPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
    skiPlowDoublePCMLock = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
  }
  public void pistonUpLeft() {
    skiPlowLeftPCM.set(false);
  }
  public void pistonUpRight() {
    skiPlowRightPCM.set(false);
  }
  public void pistonDownLeft() {
    skiPlowLeftPCM.set(true);
  }
  public void pistonDownRight() {
    skiPlowRightPCM.set(true);
  }
  public void lockOn() {
    skiPlowDoublePCMLock.set(true);
  }
  public void lockOff() {
    skiPlowDoublePCMLock.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}