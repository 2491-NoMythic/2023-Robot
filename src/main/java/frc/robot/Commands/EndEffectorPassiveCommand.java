// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.EndEffector;

public class EndEffectorPassiveCommand extends CommandBase {

  /** Creates a new EndEffectorCommand. */
  public EndEffector endEffector;
  public IntakeState intakeState;

  public EndEffectorPassiveCommand(EndEffector effector) {
    addRequirements(effector);
    this.intakeState = intakeState.getInstance();
    this.endEffector = effector;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setEndEffectorBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeState.isConeMode()) endEffector.rollerPassiveCone();
    if (intakeState.isCubeMode()) endEffector.rollerPassiveCube();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.rollerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
