// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.SkiPlow;

public class IntakeCommand extends CommandBase {
  /** Creates a new SkiPlowPneumatic. */
  public SkiPlow skiplow;
  public IntakeState intakeState;

  public IntakeCommand(SkiPlow skiPlow) {
    this.skiplow =  skiPlow;
    this.intakeState = intakeState.getInstance();
    addRequirements(skiPlow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intakeState.isConeMode()) skiplow.rollerCone();
    if (intakeState.isCubeMode()) skiplow.rollerCube();
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    skiplow.rollerOff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
