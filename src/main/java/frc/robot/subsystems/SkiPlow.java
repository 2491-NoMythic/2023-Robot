// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static frc.robot.settings.Constants.Intake.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SkiPlow extends SubsystemBase {
  /** Creates a new SkiPlow. */
  // Solenoid skiPlowLeftPCM;
  // Solenoid skiPlowDoublePCMLock;
  // Solenoid skiPlowRightPCM;
  CANSparkMax roller;
  double maxSpeed;
  DoubleSolenoid skiPlowPneumatic;
  public SkiPlow(double maxRollerSpeed) {
    // skiPlowLeftPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 5);
    // skiPlowRightPCM = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
    // skiPlowDoublePCMLock = new Solenoid(PneumaticsModuleType.CTREPCM, 3);
    roller = new CANSparkMax(INTAKE_MOTOR_ID, MotorType.kBrushless);
    this.maxSpeed = maxRollerSpeed;
    skiPlowPneumatic = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
  }
  // public void pistonUpLeft() {
  //   skiPlowLeftPCM.set(false);
  // }
  // public void pistonUpRight() {
  //   skiPlowRightPCM.set(false);
  // }
  // public void pistonDownLeft() {
  //   skiPlowLeftPCM.set(true);
  // }
  // public void pistonDownRight() {
  //   skiPlowRightPCM.set(true);
  // }
  // public void lockOn() {
  //   skiPlowDoublePCMLock.set(true);
  // }
  // public void lockOff() {
  //   skiPlowDoublePCMLock.set(false);
  // }
  public void pistonUp() {
    skiPlowPneumatic.set(Value.kReverse);
    // skiPlowLeftPCM.set(false);
    // skiPlowRightPCM.set(false);
    // this /\ code was used before a double solenoid was added

  }
  public void pistonDown() {
    skiPlowPneumatic.set(Value.kForward);
    // skiPlowLeftPCM.set(true);
    // skiPlowRightPCM.set(true);
    // this /\ code was used before a double solenoid was added
  }
  public void rollerCube() {
    roller.set(maxSpeed);
  }
  public void rollerCone() {
    roller.set(-maxSpeed);
  }
  public void rollerOff() {
    roller.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}