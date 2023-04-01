// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.settings.Constants.Poses.MID_CONE;
import static frc.robot.settings.Constants.Poses.RESET;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MidCone extends SequentialCommandGroup {
  private static final double TIMEOUT = 1.0;

  /** Creates a new MidCone. */
  public MidCone(ArmSubsystem arm, SkiPlow intake) {
    // Add your commands in the addCommands() call, e.g.
    addCommands(
        runOnce(intake::pistonDown, intake).unless(() -> !MID_CONE.isRequiresIntakeDown()),
        runOnce(() -> arm.setDesiredSholderPose(RESET), arm).unless(() -> !arm.isExtended()),
        runOnce(() -> arm.setDesiredElbowPose(MID_CONE), arm),
        waitUntil(arm::isElbowAtTarget).withTimeout(TIMEOUT),
        runOnce(() -> arm.setDesiredSholderPose(MID_CONE), arm),
        runOnce(intake::pistonUp, intake));
  }
}
