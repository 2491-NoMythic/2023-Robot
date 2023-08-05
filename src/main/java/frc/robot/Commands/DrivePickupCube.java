// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Commands.arm.IntakeCube;
import frc.robot.Commands.arm.ResetFast;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivePickupCube extends SequentialCommandGroup {
  /** Creates a new DrivePickupCube. */
  public DrivePickupCube(DrivetrainSubsystem drivetrain, ArmSubsystem arm, EndEffector intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveToCube(drivetrain), new IntakeCube(arm), new WaitCommand(3), new ResetFast(arm));
  }
}
