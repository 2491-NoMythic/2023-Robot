// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.EndEffector;

public class EndEffectorRun extends CommandBase {

  /** Creates a new EndEffectorCommand. */
  public EndEffector endEffector;
  public BooleanSupplier isIntaking;
  public IntakeState intakeState;

  public EndEffectorRun(EndEffector effector, 
    BooleanSupplier isIntaking) {
    addRequirements(effector);
    this.isIntaking = isIntaking;
    this.endEffector = effector;
    intakeState = intakeState.getInstance();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeState.isConeMode()) {
      if (isIntaking.getAsBoolean()) {
        endEffector.rollerInCone();
      } 
      else endEffector.rollerOutCone();
    }
    if (intakeState.isCubeMode()) {
      if (isIntaking.getAsBoolean()) {
        endEffector.rollerInCube();
      } 
      else endEffector.rollerOutCube();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
