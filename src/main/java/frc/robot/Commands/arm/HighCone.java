// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.settings.Constants.Poses.HIGH_CONE;
import static frc.robot.settings.Constants.Poses.RESET;
import static frc.robot.settings.Constants.Poses.AVOID_POST;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HighCone extends SequentialCommandGroup {
  private static final double TIMEOUT = 1.0;

  /** Creates a new HighCone. */
  // public HighCone(ArmSubsystem arm, SkiPlow intake) {
  public HighCone(ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        // runOnce(intake::pistonDown, intake).unless(() -> !HIGH_CONE.isRequiresIntakeDown()),
        either(
            Commands.sequence(
                runOnce(() -> arm.setDesiredSholderPose(RESET), arm),
                waitUntil(arm::isShoulderAtTarget).withTimeout(TIMEOUT)),
                none(),
                arm::isExtended), //TODO: could be a reset command ?
        runOnce(() -> arm.setDesiredElbowPose(AVOID_POST), arm),
        runOnce(() -> arm.setDesiredSholderPose(AVOID_POST), arm),
        waitUntil(arm::isShoulderAtTarget).withTimeout(TIMEOUT),
        runOnce(() -> arm.setDesiredElbowPose(HIGH_CONE), arm),
        waitUntil(arm::isElbowAtTarget).withTimeout(TIMEOUT),
        runOnce(() -> arm.setDesiredSholderPose(HIGH_CONE), arm),
        // runOnce(intake::pistonUp, intake),
        waitUntil(arm::isShoulderAtTarget).withTimeout(TIMEOUT));
  }
}
