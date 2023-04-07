// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import static frc.robot.settings.Constants.Arm.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class EndEffector extends SubsystemBase {

    CANSparkMax sparkEndEffectorSmallRoller;
    CANSparkMax sparkEndEffectorBigRoller;
    RelativeEncoder endEffectorEncoder;
    double rollerSpeedSmall;
    double rollerSpeedBig;
    
    public EndEffector(double maxSpeedBig, double maxSpeedSmall) {
        sparkEndEffectorBigRoller = new CANSparkMax(END_EFFECTOR_BIG_ROLLER_ID, MotorType.kBrushless);
        sparkEndEffectorSmallRoller = new CANSparkMax(END_EFFECTOR_SMALL_ROLLER_ID, MotorType.kBrushless);
        sparkEndEffectorSmallRoller.setInverted(false);
        sparkEndEffectorBigRoller.setIdleMode(IdleMode.kBrake);
        sparkEndEffectorBigRoller.setSmartCurrentLimit(20);
        sparkEndEffectorSmallRoller.setIdleMode(IdleMode.kBrake);
        endEffectorEncoder = sparkEndEffectorSmallRoller.getEncoder();
        endEffectorEncoder = sparkEndEffectorBigRoller.getEncoder();
        rollerSpeedSmall = maxSpeedSmall;
        rollerSpeedBig = maxSpeedBig;
        sparkEndEffectorBigRoller.burnFlash();
        sparkEndEffectorSmallRoller.burnFlash();
     }

    public void setEndEffectorBrakeMode() {
        sparkEndEffectorBigRoller.setIdleMode(IdleMode.kBrake);
        sparkEndEffectorSmallRoller.setIdleMode(IdleMode.kBrake);
    }

    public void setEndEffectorPower(double bigspeed, double smallspeed) {
        sparkEndEffectorBigRoller.set(bigspeed);
        sparkEndEffectorSmallRoller.set(smallspeed);
    }

    public void rollerInCube() {
        sparkEndEffectorBigRoller.set(rollerSpeedBig);
        sparkEndEffectorSmallRoller.set(rollerSpeedSmall); 
    }

    public void rollerOutCube() {
        sparkEndEffectorBigRoller.set(-rollerSpeedBig);
        sparkEndEffectorSmallRoller.set(-rollerSpeedSmall);
    }

    public void rollerInCone() {
        sparkEndEffectorBigRoller.set(-rollerSpeedBig);
        sparkEndEffectorSmallRoller.set(-rollerSpeedSmall);
    }

    public void rollerOutCone() {
        sparkEndEffectorBigRoller.set(rollerSpeedBig+END_EFFECTOR_CONE_POWER_BONUS);
        sparkEndEffectorSmallRoller.set(rollerSpeedSmall+END_EFFECTOR_CONE_POWER_BONUS);
    }

    public void rollerOff() {
        sparkEndEffectorBigRoller.set(0);
        sparkEndEffectorSmallRoller.set(0);
    }

    public void rollerPassiveCube() {
        sparkEndEffectorBigRoller.set(END_EFFECTOR_BIG_PASSIVE_POWER);
        sparkEndEffectorSmallRoller.set(END_EFFECTOR_SMALL_PASSIVE_POWER);
    }

    public void rollerPassiveCone() {
        sparkEndEffectorBigRoller.set(-END_EFFECTOR_BIG_PASSIVE_POWER);
        sparkEndEffectorSmallRoller.set(-END_EFFECTOR_SMALL_PASSIVE_POWER);
    }

    public double getEndEffectorPosition() {
        return endEffectorEncoder.getPosition();
    }
    
}    
