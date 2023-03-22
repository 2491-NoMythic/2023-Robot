// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;

public class EndEffectorCommand extends CommandBase {

  /** Creates a new EndEffectorCommand. */
  public EndEffector endEffector;
  public PS4Controller opController;
  public IntSupplier endEffectorAxis;
  public double speed;
  public EndEffectorCommand(EndEffector effector, 
    IntSupplier endEffectorAxisSupplier, double speed) {
    addRequirements(effector);
    this.endEffectorAxis = endEffectorAxisSupplier;
    this.endEffector = effector;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setEndEffectorBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (endEffectorAxis.getAsInt()==-1) {
      endEffector.setEndEffectorPower(0, 0);
    } 
    if (endEffectorAxis.getAsInt()==0) {
      endEffector.rollerInCone();
    } 
    if (endEffectorAxis.getAsInt()==90) {
      endEffector.rollerOutCone();
    } 
    if (endEffectorAxis.getAsInt()==180) {
      endEffector.rollerInCube();
    } 
    if (endEffectorAxis.getAsInt()==270) {
      endEffector.rollerOutCube();
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setEndEffectorPower(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
