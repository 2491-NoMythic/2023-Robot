// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;




public class ArmSubsystem extends SubsystemBase {
  CANSparkMax shoulderMotor;
  CANSparkMax elbowMotor;
  SparkMaxPIDController shoulderPID;
  SparkMaxPIDController elbowPID;
  AbsoluteEncoder shoulderEncoder;
  AbsoluteEncoder elbowEncoder;
  Solenoid shoulderLock;
  Solenoid elbowLock;
  Rotation2d[] lastAngles = {new Rotation2d(), new Rotation2d()};
  double skP = ARM_SHOULDER_K_P;
  double skI = ARM_SHOULDER_K_I;
  double skD = ARM_SHOULDER_K_D;
  double skFF = ARM_SHOULDER_FF_K_G;
  double sDeg = 0;
  double ekP = ARM_ELBOW_K_P;
  double ekI = ARM_ELBOW_K_I;
  double ekD = ARM_ELBOW_K_D;
  double ekFF = ARM_ELBOW_FF_K_G;
  double eDeg = 0;
  GenericEntry xTarget;
  GenericEntry yTarget;
  GenericEntry calculateAnglesTrue;
    
  public ArmSubsystem(){

    ShuffleboardTab tab = Shuffleboard.getTab(ARM_SHUFFLEBOARD_TAB);
    // SHOULDER
    shoulderMotor = new CANSparkMax(ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    shoulderLock = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_SHOULDER_LOCK_CHANNEL);
    
    shoulderEncoder.setPositionConversionFactor(360);
    shoulderEncoder.setVelocityConversionFactor(360);
    shoulderEncoder.setInverted(true);
    shoulderEncoder.setZeroOffset(ARM_SHOULDER_ENCODER_OFFSET_DEG);
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setInverted(true);
    shoulderMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

    shoulderPID = shoulderMotor.getPIDController();
    shoulderPID.setFeedbackDevice(shoulderEncoder);
    shoulderPID.setPositionPIDWrappingEnabled(true);
    shoulderPID.setPositionPIDWrappingMinInput(0);
    shoulderPID.setPositionPIDWrappingMaxInput(360);
    shoulderPID.setP(ARM_SHOULDER_K_P);
    shoulderPID.setI(ARM_SHOULDER_K_I);
    shoulderPID.setD(ARM_SHOULDER_K_D);
    shoulderPID.setOutputRange(-0.5, 0.5);
    shoulderMotor.burnFlash();

    // ELBOW
    elbowMotor = new CANSparkMax(ARM_ELBOW_MOTOR_ID, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    elbowLock = new Solenoid(PneumaticsModuleType.CTREPCM, ARM_ELBOW_LOCK_CHANNEL);
    
    elbowEncoder.setPositionConversionFactor(360);
    elbowEncoder.setVelocityConversionFactor(360);
    elbowEncoder.setInverted(true);
    elbowEncoder.setZeroOffset(ARM_ELBOW_ENCODER_OFFSET);
    elbowMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setInverted(false);
    elbowMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec

    elbowPID = elbowMotor.getPIDController();
    elbowPID.setFeedbackDevice(elbowEncoder);
    elbowPID.setPositionPIDWrappingEnabled(true);
    elbowPID.setPositionPIDWrappingMinInput(0);
    elbowPID.setPositionPIDWrappingMaxInput(360);
    elbowPID.setP(ARM_ELBOW_K_P);
    elbowPID.setI(ARM_ELBOW_K_I);
    elbowPID.setD(ARM_ELBOW_K_D);
    elbowPID.setOutputRange(-0.5, 0.5);
    elbowMotor.burnFlash();
    
    SmartDashboard.putNumber("Shoulder P", ARM_SHOULDER_K_P);
    SmartDashboard.putNumber("Shoulder I", ARM_SHOULDER_K_I);
    SmartDashboard.putNumber("Shoulder D", ARM_SHOULDER_K_D);
    SmartDashboard.putNumber("Shoulder Feed Forward", ARM_SHOULDER_FF_K_G);
    SmartDashboard.putNumber("Shoulder Set Degrees", 0);
    SmartDashboard.putBoolean("Run Shoulder", false);
    
    SmartDashboard.putNumber("Elbow P", ARM_ELBOW_K_P);
    SmartDashboard.putNumber("Elbow I", ARM_ELBOW_K_I);
    SmartDashboard.putNumber("Elbow D", ARM_ELBOW_K_D);
    SmartDashboard.putNumber("Elbow Feed Forward", ARM_ELBOW_FF_K_G);
    SmartDashboard.putNumber("Elbow Set Degrees", 0);
    SmartDashboard.putBoolean("Run Elbow", false);
    
    tab.addDouble("Shoulder Angle", () -> getShoulderAngle().getDegrees());
    tab.addDouble("Elbow Angle", () -> getElbowAngle().getDegrees());
    tab.addDouble("Target Shoulder Angle", () -> lastAngles[0].getDegrees());
    tab.addDouble("Target Elbow Angle", () -> lastAngles[1].getDegrees());
    tab.addBoolean("Shoulder at Target", () -> isShoulderAtTarget());
    tab.addBoolean("Elbow at Target", () -> isElbowAtTarget());
    xTarget = tab.add("X Target Setpoint", 0).getEntry();
    yTarget = tab.add("Y Target Setpoint", 0).getEntry();
    calculateAnglesTrue = tab.add("Calculate From Targets", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry(); 
    
  }
  /**
   * @return the angle of the shoulder segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromDegrees(shoulderEncoder.getPosition());
  }
  /**
   * @return the angle of the elbow segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getElbowAngle() {
    return Rotation2d.fromDegrees(elbowEncoder.getPosition());
  }
  public boolean isShoulderAtTarget() { 
    return (Math.abs(shoulderEncoder.getPosition()-lastAngles[0].getDegrees()) <= ARM_SHOULDER_ALLOWABLE_ERROR_DEG 
    || Math.abs(shoulderEncoder.getPosition()-lastAngles[0].getDegrees()) >= 360-ARM_SHOULDER_ALLOWABLE_ERROR_DEG);
  }
  public boolean isElbowAtTarget() { 
    return (Math.abs(elbowEncoder.getPosition()-lastAngles[1].getDegrees()) <= ARM_ELBOW_ALLOWABLE_ERROR_DEG 
    || Math.abs(elbowEncoder.getPosition()-lastAngles[1].getDegrees()) >= 360-ARM_ELBOW_ALLOWABLE_ERROR_DEG);
  }
  // public boolean isElbowAtTarget
  /**
   * @return the position of the end of the arm relative to the base of the shoulder joint?
   * X+ = robot forwards, Y+ = robot up. The measurements are in meters.
   */
  public Translation2d getArmPose(Rotation2d[] angles) {
    Translation2d shoulderPostion = new Translation2d(0,ARM_SHOULDER_LENGTH_METERS).rotateBy(angles[0].unaryMinus());
    Translation2d elbowPosition = new Translation2d(0,-ARM_ELBOW_LENGTH_METERS).rotateBy(angles[1]);
    Translation2d armPose = shoulderPostion.plus(elbowPosition);
    return armPose;
  }

  /**
   * @param pose
   * @return [0]Shoulder Angle, [1]Elbow Angle
   */
  public Rotation2d[] calculateJointAngles(Translation2d pose) {
    double magnitude = pose.getNorm();
    double poseVector = 90-Math.atan2(pose.getY(), pose.getX());
    double shoulderVector = Math.acos((Math.pow(ARM_SHOULDER_LENGTH_METERS,2)-Math.pow(ARM_ELBOW_LENGTH_METERS, 2)-Math.pow(magnitude, 2))/(-2*ARM_ELBOW_LENGTH_METERS*magnitude));
    double elbowVector = Math.acos((Math.pow(ARM_ELBOW_LENGTH_METERS, 2) - Math.pow(ARM_SHOULDER_LENGTH_METERS, 2) - Math.pow(magnitude, 2))/ (-2 * ARM_SHOULDER_LENGTH_METERS * magnitude));
    if (pose.getX() <= 0.0) elbowVector = -elbowVector;
    Rotation2d shoulderAngle = Rotation2d.fromDegrees(-(poseVector+shoulderVector) + 270);
    Rotation2d elbowAngle = Rotation2d.fromDegrees(poseVector-elbowVector-90);
    return new Rotation2d[] {shoulderAngle, elbowAngle};
  }
  public double[] calculateFeedForward(Rotation2d[] angles) {
    Translation2d shoulderPos= new Translation2d(0,ARM_SHOULDER_LENGTH_METERS).rotateBy(angles[0].unaryMinus());
    Translation2d comPos = new Translation2d(0,-ARM_ELBOW_CENTER_OF_MASS_OFFSET_METERS).rotateBy(angles[1]);
    Translation2d totalPos = shoulderPos.plus(comPos);
    Rotation2d comAngle = totalPos.getAngle();
    SmartDashboard.putString("Shoulder location", shoulderPos.toString());
    SmartDashboard.putString("CenterOfMass location", comPos.toString());
    SmartDashboard.putString("totalOffset location", totalPos.toString());
    SmartDashboard.putString("CenterOfMass Angle", comAngle.toString());

    double shoulderFF = totalPos.getX() * skFF;
    double elbowFF = angles[1].getSin() * ekFF;
    SmartDashboard.putNumber("Shoulder FF out", shoulderFF);
    SmartDashboard.putNumber("Elbow FF out", elbowFF);
    return new double[] {shoulderFF, elbowFF};
  }
  public void setShoulderLock(Boolean locked){
    shoulderLock.set(!locked);
  }
  public void setElbowLock(Boolean locked){
    elbowLock.set(!locked);
  }
  public void setShoulderPower(double power){
      shoulderMotor.set(power);
  }
  public void setElbowPower(double power){
      elbowMotor.set(power);
  }
  /**
   * sets the target angle for the first segment of the arm in relation to the ground
   * @param angle
   */
  public void setShoulderAngle(Rotation2d angle) {
    setShoulderAngle(angle, 0);
  }
  public void setShoulderAngle(Rotation2d angle, double feedforward) {
    shoulderPID.setReference(angle.getDegrees(), ControlType.kPosition, 0, feedforward);
    lastAngles[0] = angle;
  }

  /**
   * sets the target angle for the second segment of the arm in relation to the ground
   * @param angle
   */
  public void setElbowAngle(Rotation2d angle) {
   setElbowAngle(angle, 0);
  }
  public void setElbowAngle(Rotation2d angle, double feedforward) {
    elbowPID.setReference(angle.getDegrees(), ControlType.kPosition, 0, feedforward);
    lastAngles[1] = angle;
  }
  public void stopShoulder() {
    shoulderMotor.stopMotor();
  }
  public void stopElbow() {
    elbowMotor.stopMotor();
  }

  /**
   * sets the target x and y for the endeffector
   * @param targetPose
   */
  public void setTarget(Translation2d targetPose) {
    Rotation2d[] angles = calculateJointAngles(targetPose);
    // setShoulderAngle(angles[0]);
    // setElbowAngle(angles[1]);
  }
  @Override
	public void periodic() {
    double new_skP = SmartDashboard.getNumber("Shoulder P", ARM_SHOULDER_K_P);
    double new_skI = SmartDashboard.getNumber("Shoulder I", ARM_SHOULDER_K_I);
    double new_skD = SmartDashboard.getNumber("Shoulder D", ARM_SHOULDER_K_D);
    double new_skFF = SmartDashboard.getNumber("Shoulder Feed Forward", ARM_SHOULDER_FF_K_G);
    double new_sDegrees = SmartDashboard.getNumber("Shoulder Set Degrees", 0);
    
    double new_ekP = SmartDashboard.getNumber("Elbow P", ARM_ELBOW_K_P);
    double new_ekI = SmartDashboard.getNumber("Elbow I", ARM_ELBOW_K_I);
    double new_ekD = SmartDashboard.getNumber("Elbow D", ARM_ELBOW_K_D);
    double new_ekFF = SmartDashboard.getNumber("Elbow Feed Forward", ARM_ELBOW_FF_K_G);
    double new_eDegrees = SmartDashboard.getNumber("Elbow Set Degrees", 0);
    
    if((new_skP!= skP)) {shoulderPID.setP(new_skP); skP=new_skP;}
    if((new_skI!= skI)) {shoulderPID.setI(new_skI); skI=new_skI;}
    if((new_skD!= skD)) {shoulderPID.setP(new_skD); skD=new_skD;}
    if((new_skFF!= skFF)) {skFF=new_skFF;}

    if((new_ekP!= ekP)) {elbowPID.setP(new_ekP); ekP=new_ekP;}
    if((new_ekI!= ekI)) {elbowPID.setI(new_ekI); ekI=new_ekI;}
    if((new_ekD!= ekD)) {elbowPID.setP(new_ekD); ekD=new_ekD;}
    if((new_ekFF!= ekFF)) {ekFF=new_ekFF;}
    double[] feedforward = calculateFeedForward(new Rotation2d[] {Rotation2d.fromDegrees(new_sDegrees), Rotation2d.fromDegrees(new_eDegrees)});
    if (SmartDashboard.getBoolean("Run Shoulder", false)) { 
      setShoulderLock(false);
      setShoulderAngle(Rotation2d.fromDegrees(new_sDegrees), feedforward[0]);
    } else {
      stopShoulder();
      setShoulderLock(true);
    }
    if (SmartDashboard.getBoolean("Run Elbow", false)) { 
      setElbowLock(false);
      setElbowAngle(Rotation2d.fromDegrees(new_eDegrees), feedforward[1]);
    } else {
      stopElbow();
      setElbowLock(true);
    }
    SmartDashboard.putString("armPose", getArmPose(lastAngles).toString());
    if (calculateAnglesTrue.getBoolean(false)) {
      setTarget(new Translation2d(xTarget.getDouble(0),yTarget.getDouble(0)));
      calculateAnglesTrue.setBoolean(false);
    }

  }
}
