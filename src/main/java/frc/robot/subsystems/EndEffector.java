// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Arm.*;

/** Add your docs here. */
public class EndEffector extends SubsystemBase {

private static final int END_EFFECTOR_MOTOR_ID = 2491;
TalonSRX endEffectorMotor;

public void EndEffectorSubsystem() {
    endEffectorMotor = new TalonSRX(END_EFFECTOR_MOTOR_ID);
    endEffectorMotor.setNeutralMode(NeutralMode.Brake);
}
public void setEndEffectorPower(double speed) {
    endEffectorMotor.set(ControlMode.PercentOutput, speed);
}
public void getEndEffectorPosition() {
    endEffectorMotor.getSelectedSensorPosition();
}



}
