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

// TalonSRX endEffectorMotor;
CANSparkMax sparkEndEffector;
RelativeEncoder endEffectorEncoder;

public EndEffector() {
    sparkEndEffector = new CANSparkMax(END_EFFECTOR_MOTOR_ID, MotorType.kBrushless);
    sparkEndEffector.setIdleMode(IdleMode.kBrake);
    endEffectorEncoder = sparkEndEffector.getEncoder();
    // endEffectorMotor = new TalonSRX(END_EFFECTOR_MOTOR_ID);
    // endEffectorMotor.setNeutralMode(NeutralMode.Brake);
}
public void setEndEffectorBrakeMode() {
    sparkEndEffector.setIdleMode(IdleMode.kBrake);
    // endEffectorMotor.setNeutralMode(NeutralMode.Brake);
}
public void setEndEffectorPower(double speed) {
    sparkEndEffector.set(speed);
    // endEffectorMotor.set(ControlMode.PercentOutput, speed);
}
public double getEndEffectorPosition() {
    return endEffectorEncoder.getPosition();
    // endEffectorMotor.getSelectedSensorPosition();
}



}
