// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class MoveToPose extends CommandBase {
  private DrivetrainSubsystem drivetrain;
  private Timer timer = new Timer();
  private double timeout;
  private Pose2d currentPose;
  private Pose2d targetPose;
  private Autos autos = Autos.getInstance();
  /** Creates a new MoveToPose. */
  public MoveToPose(DrivetrainSubsystem drivetrain, Pose2d targetPose) {
    this.drivetrain = drivetrain;
    this.targetPose = targetPose;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    currentPose = drivetrain.getPose();
    PathPlannerTrajectory trajectory = autos.calculatePath(currentPose, targetPose);
    timeout = trajectory.getTotalTimeSeconds();
    autos.followPath(trajectory);
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(timeout);
  }
}
