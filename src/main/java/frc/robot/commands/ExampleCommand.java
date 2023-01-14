// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ExampleCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final ExampleSubsystem dependency;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ExampleCommand(ExampleSubsystem subsystem) {
    dependency = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { /* TODO document why this method is empty or remove */ }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { /* TODO document why this method is empty or remove */ }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) { /* TODO document why this method is empty or remove */ }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
