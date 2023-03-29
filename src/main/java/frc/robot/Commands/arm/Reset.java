// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.either;
import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.settings.Constants.Poses.RESET;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Reset extends SequentialCommandGroup {
  /** Creates a new Reset. */
  public Reset(ArmSubsystem arm, SkiPlow intake) {

    addCommands(
        either(runOnce(intake::pistonDown, intake), none(), RESET::isRequiresIntakeDown),
        runOnce(() -> arm.setDesiredElbowPose(RESET), arm),
        waitUntil(arm::isElbowAtTarget).withTimeout(1),
        runOnce(() -> arm.setDesiredSholderPose(RESET), arm),
        runOnce(intake::pistonUp, intake));
  }
}
