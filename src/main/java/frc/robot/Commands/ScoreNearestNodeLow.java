
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.arm.DropLow;
import frc.robot.Commands.arm.Reset;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreNearestNodeLow extends SequentialCommandGroup {
  /** Creates a new ScoreNearestNodeLow. */
  public ScoreNearestNodeLow(DrivetrainSubsystem drivetrain, ArmSubsystem arm, EndEffector endEffector) {

    addCommands(
        Commands.either(
            new SequentialCommandGroup(
                new MoveToPose(drivetrain, drivetrain.getNearestNode()),
                Commands.waitSeconds(0.25),
                new DropLow(arm),
                new EndEffectorCommand(endEffector, () -> false).withTimeout(1),
                new Reset(arm)),
            new SequentialCommandGroup(Commands.none()),
            () -> (drivetrain.getPose().getTranslation()
                .getDistance(drivetrain.getNearestNode().getTranslation()) < 1)));
  }
}
