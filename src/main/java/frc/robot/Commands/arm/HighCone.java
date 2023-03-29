// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static frc.robot.settings.Constants.Poses.HIGH_CONE;

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
        either(runOnce(intake::pistonDown, intake), none(), HIGH_CONE::isRequiresIntakeDown),
        runOnce(intake::pistonUp, intake),
        runOnce(() -> arm.setDesiredElbowPose(HIGH_CONE), arm),
        runOnce(() -> arm.setDesiredSholderPose(HIGH_CONE), arm),
        runOnce(intake::pistonUp, intake));
  }
}
