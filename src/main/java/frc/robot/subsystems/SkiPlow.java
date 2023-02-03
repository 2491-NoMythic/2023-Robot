// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

public class SkiPlow extends SubsystemBase {
  /** Creates a new SkiPlow. */
  DoubleSolenoid skiPlowDoublePCM;
  DoubleSolenoid skiPlowDoublePCMLock;
  public SkiPlow() {
    skiPlowDoublePCM = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 1, 0);
    skiPlowDoublePCMLock = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 2);
  }
  public void pistonReverse() {
    skiPlowDoublePCM.set(kReverse);
  }
  public void pistonForward() {
    skiPlowDoublePCM.set(kForward);
  }
  public void pistonoff() {
    skiPlowDoublePCM.set(kOff);
  }
  
  public void lockReverse() {
    skiPlowDoublePCMLock.set(kReverse);
  }
  public void lockForward() {
    skiPlowDoublePCMLock.set(kForward);
  }
  public void lockoff() {
    skiPlowDoublePCMLock.set(kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}