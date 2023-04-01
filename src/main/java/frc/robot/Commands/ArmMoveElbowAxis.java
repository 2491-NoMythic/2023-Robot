// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveElbowAxis extends CommandBase {
  /** Creates a new test. */
  private ArmSubsystem arm;
  private DoubleSupplier axis;
  public ArmMoveElbowAxis(ArmSubsystem arm, DoubleSupplier axis) {
    this.arm = arm;
    this.axis = axis;
    addRequirements(arm);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!(MathUtil.applyDeadband(axis.getAsDouble(), 0.05)==0)) {
      arm.setDesiredElbowRotation(arm.getElbowTarget().plus(Rotation2d.fromDegrees(-axis.getAsDouble()*0.4)));
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

