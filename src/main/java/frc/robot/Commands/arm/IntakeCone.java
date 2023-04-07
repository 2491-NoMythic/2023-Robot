// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
import static frc.robot.settings.Constants.Poses.INTAKE_CONE;
import static frc.robot.settings.Constants.Poses.AVOID_BUMPER;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCone extends SequentialCommandGroup {
  /** Creates a new IntakeCone. */
  // public IntakeCone(ArmSubsystem arm, SkiPlow intake) {
  public IntakeCone(ArmSubsystem arm) {

    addCommands(
        // runOnce(intake::pistonDown, intake).unless(() -> !INTAKE_CONE.isRequiresIntakeDown()),
        // new IntakeCommand(intake),
        // new WaitCommand(0.5),
        runOnce(() -> arm.setDesiredElbowPose(AVOID_BUMPER), arm),
        runOnce(() -> arm.setDesiredSholderPose(AVOID_BUMPER), arm),
        waitUntil(arm::isShoulderAtTarget).withTimeout(1),
        runOnce(() -> arm.setDesiredSholderPose(INTAKE_CONE), arm),
        runOnce(() -> arm.setDesiredElbowPose(INTAKE_CONE), arm));
  }
}
