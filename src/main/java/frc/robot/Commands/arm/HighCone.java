// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static frc.robot.settings.Constants.armPoses.HIGH_CONE;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighCone extends SequentialCommandGroup {
  /** Creates a new HighCone. */
  public HighCone(ArmSubsystem arm, SkiPlow intake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        new InstantCommand(intake::pistonUp, intake),
        new InstantCommand(() -> arm.setDesiredElbowAngle(HIGH_CONE[1]), arm),
        new InstantCommand(() -> arm.setDesiredSholderAngle(HIGH_CONE[0]), arm));
  }
}
