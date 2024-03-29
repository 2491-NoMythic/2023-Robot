// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.settings.Constants.Arm.ARM_ELBOW_ALLOWABLE_ERROR_DEG;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_CENTER_OF_MASS_OFFSET_METERS;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_ENCODER_OFFSET;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_FF_K_G;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_K_D;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_K_I;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_K_P;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_LENGTH_METERS;
import static frc.robot.settings.Constants.Arm.ARM_ELBOW_MOTOR_ID;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_ALLOWABLE_ERROR_DEG;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_ENCODER_OFFSET_DEG;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_FF_K_G;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_K_D;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_K_I;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_K_P;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_LENGTH_METERS;
import static frc.robot.settings.Constants.Arm.ARM_SHOULDER_MOTOR_ID;
import static frc.robot.settings.Constants.Arm.ARM_SHUFFLEBOARD_TAB;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.Poses;

public class ArmSubsystem extends SubsystemBase {
  CANSparkMax shoulderMotor;
  CANSparkMax elbowMotor;
  SparkMaxPIDController shoulderPID;
  SparkMaxPIDController elbowPID;
  AbsoluteEncoder shoulderEncoder;
  AbsoluteEncoder elbowEncoder;
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
  // GenericEntry xTarget;
  // GenericEntry yTarget;
  // GenericEntry calculateAnglesTrue;
    
  public ArmSubsystem(){

    ShuffleboardTab tab = Shuffleboard.getTab(ARM_SHUFFLEBOARD_TAB);
    // SHOULDER
    shoulderMotor = new CANSparkMax(ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless);
    shoulderMotor.restoreFactoryDefaults();
    shoulderEncoder = shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    
    shoulderEncoder.setPositionConversionFactor(360);
    shoulderEncoder.setVelocityConversionFactor(360);
    shoulderEncoder.setInverted(true);
    shoulderEncoder.setZeroOffset(ARM_SHOULDER_ENCODER_OFFSET_DEG);
    shoulderMotor.setIdleMode(IdleMode.kBrake);
    shoulderMotor.setInverted(true);
    shoulderMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec
    shoulderMotor.setClosedLoopRampRate(0.25);

    shoulderPID = shoulderMotor.getPIDController();
    shoulderPID.setFeedbackDevice(shoulderEncoder);
    shoulderPID.setPositionPIDWrappingEnabled(true);
    shoulderPID.setPositionPIDWrappingMinInput(0);
    shoulderPID.setPositionPIDWrappingMaxInput(360);
    shoulderPID.setP(ARM_SHOULDER_K_P);
    shoulderPID.setI(ARM_SHOULDER_K_I);
    shoulderPID.setD(ARM_SHOULDER_K_D);
    shoulderPID.setOutputRange(-0.6, 0.6);
    shoulderMotor.burnFlash();

    // ELBOW
    elbowMotor = new CANSparkMax(ARM_ELBOW_MOTOR_ID, MotorType.kBrushless);
    elbowMotor.restoreFactoryDefaults();
    elbowEncoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);

    elbowEncoder.setPositionConversionFactor(360);
    elbowEncoder.setVelocityConversionFactor(360);
    elbowEncoder.setInverted(true);
    elbowEncoder.setZeroOffset(ARM_ELBOW_ENCODER_OFFSET);
    elbowMotor.setIdleMode(IdleMode.kBrake);
    elbowMotor.setInverted(false);
    elbowMotor.setSmartCurrentLimit(60); //max currrent rating not exceed 60A or 100A more than 2 sec
    elbowMotor.setClosedLoopRampRate(0.25);

    elbowPID = elbowMotor.getPIDController();
    elbowPID.setFeedbackDevice(elbowEncoder);
    elbowPID.setPositionPIDWrappingEnabled(true);
    elbowPID.setPositionPIDWrappingMinInput(0);
    elbowPID.setPositionPIDWrappingMaxInput(360);
    elbowPID.setP(ARM_ELBOW_K_P);
    elbowPID.setI(ARM_ELBOW_K_I);
    elbowPID.setD(ARM_ELBOW_K_D);
    elbowPID.setOutputRange(-0.6, 0.6);
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
   
    //start arm in current position
    lastAngles[0] = getShoulderAngle();
    lastAngles[1] = getElbowAngle();
    tab.addBoolean("isExtened", this::isExtended).withSize(9,1).withPosition(1,4);
    tab.addDouble("Shoulder Angle", () -> getShoulderAngle().getDegrees()).withSize(2,1).withPosition(1,1);
    tab.addDouble("Elbow Angle", () -> getElbowAngle().getDegrees()).withSize(2,1).withPosition(6,1);
    tab.addDouble("Target Shoulder Angle", () -> lastAngles[0].getDegrees()).withSize(2,1).withPosition(1,2);
    tab.addDouble("Target Elbow Angle", () -> lastAngles[1].getDegrees()).withSize(2,1).withPosition(6,2);
    tab.addBoolean("Shoulder at Target", () -> isShoulderAtTarget()).withSize(1,2).withPosition(3,1);
    tab.addBoolean("Elbow at Target", () -> isElbowAtTarget()).withSize(1,2).withPosition(8,1);
    tab.addDoubleArray("Arm Pose", ()-> new double[]{getArmPose().getX(),getArmPose().getY()}).withSize(2,1).withPosition(4,3);
    // xTarget = tab.add("X Target Setpoint", 0).getEntry();
    // yTarget = tab.add("Y Target Setpoint", 0).getEntry();
    // calculateAnglesTrue = tab.add("Calculate From Targets", false).withWidget(BuiltInWidgets.kToggleSwitch).getEntry(); 
  }
  /**
   * @return the angle of the shoulder segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getShoulderAngle() {
    return Rotation2d.fromDegrees(MathUtil.inputModulus(shoulderEncoder.getPosition(),-180, 180));
  }
  /**
   * @return the angle of the elbow segment in relation to the ground. 
   * The angle is positive when the end of the segment is tilted towards the front of the robot.
   */
  public Rotation2d getElbowAngle() {
    return Rotation2d.fromDegrees(MathUtil.inputModulus(elbowEncoder.getPosition(),-180, 180));
  }
  public Rotation2d getShoulderTarget() {
    return lastAngles[0];
  }
  public Rotation2d getElbowTarget() {
    return lastAngles[1];
  }
  public boolean isShoulderWithinBounds(double boundDegrees) { 
    return Math.abs(getShoulderAngle().getDegrees()-lastAngles[0].getDegrees()) <= boundDegrees;
    // return (Math.abs(shoulderEncoder.getPosition()-lastAngles[0].getDegrees()) <= boundDegrees
    // || Math.abs(shoulderEncoder.getPosition()-lastAngles[0].getDegrees()) >= 360-boundDegrees);
  }
  public boolean isElbowWithinBounds(double boundDegrees) { 
    return Math.abs(getElbowAngle().getDegrees()-lastAngles[1].getDegrees()) <= boundDegrees;
    // return (Math.abs(elbowEncoder.getPosition()-lastAngles[1].getDegrees()) <= boundDegrees 
    // || Math.abs(elbowEncoder.getPosition()-lastAngles[1].getDegrees()) >= 360-boundDegrees);
  }
  public boolean isShoulderAtTarget() { 
    return isShoulderWithinBounds(ARM_SHOULDER_ALLOWABLE_ERROR_DEG); 
  }
  public boolean isElbowAtTarget() { 
    return isElbowWithinBounds(ARM_ELBOW_ALLOWABLE_ERROR_DEG);
  }
  public boolean isExtended() {
    double elbowPos = Math.abs(getElbowAngle().getDegrees());
    double sholderPos = Math.abs(getShoulderAngle().getDegrees());
    return 
        ((elbowPos > 10) && 
        (elbowPos < 350)) || 
        ((sholderPos > 5) && 
        (sholderPos < 355)); //TODO: double check values
  }
  public Rotation2d[] getArmAngles() {
    return new Rotation2d[] {getShoulderAngle(), getElbowAngle()};
  }
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
  public Translation2d getArmPose() {
    return getArmPose(getArmAngles());
  }

  // /**
  //  * @param pose
  //  * @return [0]Shoulder Angle, [1]Elbow Angle
  //  */
  // public Rotation2d[] calculateJointAngles(Translation2d pose) {
  //   double magnitude = pose.getNorm();
  //   double poseVector = 90-Math.atan2(pose.getY(), pose.getX());
  //   double shoulderVector = Math.acos((Math.pow(ARM_SHOULDER_LENGTH_METERS,2)-Math.pow(ARM_ELBOW_LENGTH_METERS, 2)-Math.pow(magnitude, 2))/(-2*ARM_ELBOW_LENGTH_METERS*magnitude));
  //   double elbowVector = Math.acos((Math.pow(ARM_ELBOW_LENGTH_METERS, 2) - Math.pow(ARM_SHOULDER_LENGTH_METERS, 2) - Math.pow(magnitude, 2))/ (-2 * ARM_SHOULDER_LENGTH_METERS * magnitude));
  //   if (pose.getX() <= 0.0) elbowVector = -elbowVector;
  //   // SmartDashboard.putNumber("shouldervector", shoulderVector);
  //   Rotation2d shoulderAngle = Rotation2d.fromDegrees(-(poseVector+shoulderVector) + 270);
  //   Rotation2d elbowAngle = Rotation2d.fromDegrees(poseVector-elbowVector-90);
  //   return new Rotation2d[] {shoulderAngle, elbowAngle};
  // }
  public double[] calculateFeedForward(Rotation2d[] angles) {
    Translation2d shoulderPos= new Translation2d(0,ARM_SHOULDER_LENGTH_METERS).rotateBy(angles[0].unaryMinus());
    Translation2d comPos = new Translation2d(0,-ARM_ELBOW_CENTER_OF_MASS_OFFSET_METERS).rotateBy(angles[1]);
    Translation2d totalPos = shoulderPos.plus(comPos);
    // Rotation2d comAngle = totalPos.getAngle();
    // SmartDashboard.putString("Shoulder location", shoulderPos.toString());
    // SmartDashboard.putString("CenterOfMass location", comPos.toString());
    // SmartDashboard.putString("totalOffset location", totalPos.toString());
    // SmartDashboard.putString("CenterOfMass Angle", comAngle.toString());

    double shoulderFF = totalPos.getX() * skFF;
    double elbowFF = angles[1].getSin() * ekFF;
    // SmartDashboard.putNumber("Shoulder FF out", shoulderFF);
    // SmartDashboard.putNumber("Elbow FF out", elbowFF);
    return new double[] {shoulderFF, elbowFF};
  }
  
  public void setShoulderPower(double power){
      shoulderMotor.set(power);
  }
  public void setElbowPower(double power){
      elbowMotor.set(power);
  }

  /** set target for the sholder to a given pose */
  public void setDesiredSholderPose(Poses pose) {
    setDesiredSholderRotation(pose.getSholder());
  }

  /**
   * sets the target rotation 2d for the first segment of the arm assuming a 12 o'clock zero and clockwise positive rotation
   * @param rot rotation2d target
   */
  public void setDesiredSholderRotation(Rotation2d rot) {
    //TODO: valid input check
    lastAngles[0] = Rotation2d.fromDegrees(MathUtil.inputModulus(rot.getDegrees(),-180, 180));
  }

  private void setShoulderAngle(Rotation2d angle, double feedforward) {
    shoulderPID.setReference(angle.getDegrees(), ControlType.kPosition, 0, feedforward);
  }
  /** set target for the elbow to a given pose */
  public void setDesiredElbowPose(Poses pose) {
    setDesiredElbowRotation(pose.getElbow());
    // SmartDashboard.putNumber("elbow debug", pose.getElbow().getDegrees());
  }

  /**
   * sets the target rotation 2d for the second segment of the arm assuming a 6 o'clock zero and counter clockwise positive rotation
   * @param rot rotation2d target
   */
  public void setDesiredElbowRotation(Rotation2d rot) {
    //TODO: valid input check
    lastAngles[1] = Rotation2d.fromDegrees(MathUtil.inputModulus(rot.getDegrees(),-180, 180));
  }

  private void setElbowAngle(Rotation2d angle, double feedforward) {
    elbowPID.setReference(angle.getDegrees(), ControlType.kPosition, 0, feedforward);
  }


  public void stopShoulder() {
    shoulderMotor.stopMotor();
  }
  public void stopElbow() {
    elbowMotor.stopMotor();
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
    
    if (SmartDashboard.getBoolean("Run Shoulder", false)) { 
      setDesiredSholderRotation(Rotation2d.fromDegrees(new_sDegrees));
      SmartDashboard.putBoolean("Run Shoulder", false);
    }
    if (SmartDashboard.getBoolean("Run Elbow", false)) { 
      setDesiredElbowRotation(Rotation2d.fromDegrees(new_eDegrees));
      SmartDashboard.putBoolean("Run Elbow", false);
    }

    double[] feedforward = calculateFeedForward(lastAngles);
    setShoulderAngle(lastAngles[0], feedforward[0]);
    setElbowAngle(lastAngles[1], feedforward[1]);
  }
}
