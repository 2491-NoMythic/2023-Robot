// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.settings.Constants.Poses.RESET;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetFast extends SequentialCommandGroup {
  private static final double TIMEOUT = 1.0;

  /** Creates a new Reset. */
//   public ResetFast(ArmSubsystem arm, SkiPlow intake) {
  public ResetFast(ArmSubsystem arm) {

    addCommands(
        // runOnce(intake::pistonDown, intake).unless(() -> !RESET.isRequiresIntakeDown()),
        either(
            Commands.sequence(
                runOnce(() -> arm.setDesiredSholderPose(RESET), arm),
                // waitUntil(arm::isShoulderAtTarget).withTimeout(TIMEOUT),
                runOnce(() -> arm.setDesiredElbowPose(RESET), arm)),
            Commands.sequence(
                runOnce(() -> arm.setDesiredElbowPose(RESET), arm),
                waitUntil(arm::isElbowAtTarget).withTimeout(TIMEOUT),
                runOnce(() -> arm.setDesiredSholderPose(RESET), arm)),
            arm::isExtended)
        // runOnce(intake::rollerOff, intake),
        // runOnce(intake::pistonUp, intake)
        // waitUntil(arm::isShoulderAtTarget),
        // waitUntil(arm::isElbowAtTarget).withTimeout(TIMEOUT)
        );


  }
}
