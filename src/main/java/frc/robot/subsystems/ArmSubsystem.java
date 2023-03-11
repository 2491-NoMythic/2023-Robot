// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;



public class ArmSubsystem extends SubsystemBase {
  CANSparkMax armShoulderMotor;
  CANSparkMax armElbowMotor;
  SparkMaxPIDController elbowPID;
  SparkMaxPIDController shoulderPID;
  SparkMaxAbsoluteEncoder shoulderEncoder;
  SparkMaxAbsoluteEncoder elbowEncoder;
  
  public ArmSubsystem(){
    armShoulderMotor = new CANSparkMax(ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless);
    armElbowMotor = new CANSparkMax(ARM_ELBOW_MOTOR_ID, MotorType.kBrushless);
    elbowPID = armElbowMotor.getPIDController();
    shoulderPID = armShoulderMotor.getPIDController();

      //shoulder encoder
      //TODO add encoders
      // shoulderEncoder = armShoulderMotor.getAbsoluteEncoder(SparkMaxRelativeEncoder.Type.kQuadrature);
      //elbow encoder
      // elbowEncoder = armElbowMotor.getAbsoluteEncoder(null);
  }
  /**
   * @return the angle of the shoulder segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getShoulderAngle() {
    return new Rotation2d();
  }
  /**
   * @return the angle of the elbow segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getElbowAngle() {
    return new Rotation2d();
  }
  /**
   * @return the position of the end of the arm relative to the base of the shoulder joint?
   * X+ = robot forwards, Y+ = robot up. The measurements are in meters.
   */
  public Transform2d getArmPose() {
    return new Transform2d();
  }

  /**
   * @param pose
   * @return [0]Shoulder Angle, [1]Elbow Angle
   */
  public Rotation2d[] calculateJointAngles(Transform2d pose) {
    //super fancy math goes here :)
    return new Rotation2d[] {new Rotation2d(), new Rotation2d()};
  }
  
  public void setShoulderPower(double power){
      armShoulderMotor.set(power);
  }
  public void setElbowPower(double power){
      armElbowMotor.set(power);
  }
  public void setBrakeMode() {
      armShoulderMotor.setIdleMode(IdleMode.kBrake);
      armElbowMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * sets the target angle for the first segment of the arm in relation to the ground
   * @param angle
   */
  public void setShoulderAngle(Rotation2d angle) {}

  /**
   * sets the target angle for the second segment of the arm in relation to the ground
   * @param angle
   */
  public void setElbowAngle(Rotation2d angle) {}

  /**
   * sets the target x and y for the endeffector
   * @param targetPose
   */
  public void setTarget(Transform2d targetPose) {

  }
}
