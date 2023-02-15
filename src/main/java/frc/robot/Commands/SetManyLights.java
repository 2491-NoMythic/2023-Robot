// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemLights;

public class SetManyLights extends CommandBase {
  SubsystemLights lights;
  int min;
  int max;
  int R;
  int G;
  int B;
  /** Creates a new SetManyLight. */
  public SetManyLights(SubsystemLights LS, int minLight, int maxLight, int red, int green, int blue) {
    addRequirements(LS);
    lights = LS;
    min = minLight;
    max = maxLight;
    R = red;
    G = green;
    B = blue;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
