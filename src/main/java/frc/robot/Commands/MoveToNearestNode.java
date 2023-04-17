// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveToNearestNode extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private Pose2d targetPose;
  private boolean limitAlliance;
  private boolean limitGamepiece;
  private CommandBase pathCommand;
  /** Creates a new MoveToNearestNode. 
   * @param limitAlliance True if nodes are limited to current alliance color.
   * @param limitGamepiece True if nodes are limited to current intake mode.
  */
  public MoveToNearestNode(DrivetrainSubsystem drivetrain, boolean limitAlliance, boolean limitGamepiece) {
    this.drivetrain = drivetrain;
    this.limitAlliance = limitAlliance;
    this.limitGamepiece = limitGamepiece;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (limitAlliance) {
      if (limitGamepiece) {
        targetPose = drivetrain.getNearestNode();
      } else {
        targetPose = drivetrain.getNearestNodeAnyMode();
      }
    } else if (limitGamepiece) {
      targetPose = drivetrain.getNearestNodeAnyAlliance();
    } else {
      targetPose = drivetrain.getNearestNodeAny();
    }

    pathCommand = new MoveToPose(drivetrain, targetPose);
    pathCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pathCommand.isFinished();
  }
}
