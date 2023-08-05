// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.arm.IntakeCube;
import frc.robot.Commands.arm.ResetFast;
import frc.robot.settings.IntakeState;
import frc.robot.settings.IntakeState.IntakeMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivePickupCube2 extends SequentialCommandGroup {
  /** Creates a new DrivePickupCube. */
  public DrivePickupCube2(DrivetrainSubsystem drivetrain, ArmSubsystem arm, EndEffector effector) {
    addCommands(
      new InstantCommand(()->IntakeState.setIntakeMode(IntakeMode.CUBE_GROUND)),
      new DriveToCube2(drivetrain),
      new EndEffectorRun(effector, ()->true),
      new IntakeCube(arm),
      new WaitCommand(2),
      new InstantCommand(effector::rollerOff, effector),
      new ResetFast(arm));
  }
}
