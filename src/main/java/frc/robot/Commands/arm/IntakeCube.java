// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.settings.Constants.Poses.AVOID_BUMPER;
import static frc.robot.settings.Constants.Poses.INTAKE_CUBE;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCube extends SequentialCommandGroup {
  /** Creates a new IntakeCube. */
  // public IntakeCube(ArmSubsystem arm, SkiPlow intake) {
  public IntakeCube(ArmSubsystem arm) {
    addCommands(
      // runOnce(intake::pistonDown, intake).unless(() -> !INTAKE_CUBE.isRequiresIntakeDown()),
      // new IntakeCommand(intake),
      // waitSeconds(0.5),
      runOnce(() -> arm.setDesiredElbowPose(AVOID_BUMPER), arm),
      runOnce(() -> arm.setDesiredSholderPose(AVOID_BUMPER), arm),
      waitUntil(arm::isShoulderAtTarget).withTimeout(1),
      runOnce(() -> arm.setDesiredSholderPose(INTAKE_CUBE), arm),
      runOnce(() -> arm.setDesiredElbowPose(INTAKE_CUBE), arm));
  }
}
