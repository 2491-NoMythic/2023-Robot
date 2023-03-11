// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */public class EndEffector extends SubsystemBase {
    private static final int END_EFFECTOR_MOTOR_1_ID = 4;
    private static final int END_EFFECTOR_MOTOR_2_ID = 5;
    
CANSparkMax sparkEndEffector1;
CANSparkMax sparkEndEffector2;
RelativeEncoder endEffectorEncoder;

public EndEffector() {
    sparkEndEffector1 = new CANSparkMax(END_EFFECTOR_MOTOR_1_ID, MotorType.kBrushless);
    sparkEndEffector2 = new CANSparkMax(END_EFFECTOR_MOTOR_2_ID, MotorType.kBrushless);

    sparkEndEffector1.setIdleMode(IdleMode.kBrake);
    sparkEndEffector2.setIdleMode(IdleMode.kBrake);

    endEffectorEncoder = sparkEndEffector1.getEncoder();
}
public void setEndEffectorBrakeMode() {
    sparkEndEffector1.setIdleMode(IdleMode.kBrake);
    sparkEndEffector2.setIdleMode(IdleMode.kBrake);
}
public void setEndEffectorPower1(double speed) {
    sparkEndEffector1.set(speed);
}
public void setEndEffectorPower2(double speed) {
    sparkEndEffector1.set(speed);
}
public void setEndEffectorPowerAll(double speed) {
    sparkEndEffector1.set(speed);
    sparkEndEffector2.set(speed);
}
public double getEndEffectorPosition() {
    return endEffectorEncoder.getPosition();
}



}
