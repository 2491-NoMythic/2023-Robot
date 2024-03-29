// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;
import static edu.wpi.first.wpilibj2.command.Commands.waitUntil;
import static frc.robot.settings.Constants.Poses.RAMP_CONE;
import static frc.robot.settings.Constants.Poses.RESET;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RampCone extends SequentialCommandGroup {
  private static final double TIMEOUT = 1.0;

  /** Creates a new ChuteCone. */
  public RampCone(ArmSubsystem arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      runOnce(() -> arm.setDesiredSholderPose(RESET), arm).unless(() -> !arm.isExtended()),
      runOnce(() -> arm.setDesiredSholderPose(RAMP_CONE), arm),
      waitUntil(arm::isShoulderAtTarget).withTimeout(TIMEOUT),
      runOnce(() -> arm.setDesiredElbowPose(RAMP_CONE), arm)
    );
  }
}
