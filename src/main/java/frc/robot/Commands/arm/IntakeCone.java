// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static frc.robot.settings.Constants.armPoses.INTAKE_CONE;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SkiPlow;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeCone extends SequentialCommandGroup {
  /** Creates a new IntakeCone. */
  public IntakeCone(ArmSubsystem arm, SkiPlow intake, Rotation2d[] pose) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(intake::pistonDown, intake), 
      new InstantCommand(() -> arm.setDesiredElbowAngle(INTAKE_CONE[1]), arm), 
      new WaitCommand(1),
      new InstantCommand(() -> arm.setDesiredSholderAngle(INTAKE_CONE[0]), arm)
      );
  }
}
