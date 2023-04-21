// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveAxis extends CommandBase {
  /** Creates a new test. */
  private ArmSubsystem arm;
  private DoubleSupplier shoulderAxis;
  private DoubleSupplier elbowAxis;
  public ArmMoveAxis(ArmSubsystem arm, DoubleSupplier shoulderAxis, DoubleSupplier elbowAxis) {
    this.arm = arm;
    this.shoulderAxis = shoulderAxis;
    this.elbowAxis = elbowAxis;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(MathUtil.applyDeadband(shoulderAxis.getAsDouble(), 0.1)==0)) {
      arm.setDesiredSholderRotation(arm.getShoulderTarget().plus(Rotation2d.fromDegrees(-shoulderAxis.getAsDouble()*0.25)));
    }
    if (!(MathUtil.applyDeadband(elbowAxis.getAsDouble(), 0.1)==0)) {
      arm.setDesiredElbowRotation(arm.getElbowTarget().plus(Rotation2d.fromDegrees(-elbowAxis.getAsDouble()*0.4)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

