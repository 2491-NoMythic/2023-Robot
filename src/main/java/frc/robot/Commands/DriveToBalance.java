// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToBalance extends CommandBase {
  /** Creates a new DriveToBalance. */
  private DrivetrainSubsystem drivetrain;
  private boolean moveForwards;
  private double movementSpeed;
  private double targetTilt = 15.0;
  private double targetTime = 15;
  private double time;
  // private boolean hasTipped;
  // private double gyroDeadzone = 5.0;
  public DriveToBalance(DrivetrainSubsystem drivetrain, boolean moveForwards) {
    this.drivetrain = drivetrain;
    this.moveForwards = moveForwards;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (moveForwards) {
      movementSpeed = 2.3;
    }
    else movementSpeed = -2.3;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(drivetrain.getGyroscopePitch().getDegrees()) >= targetTilt) time++;
    drivetrain.drive(new ChassisSpeeds(movementSpeed, 0, 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return time >= targetTime;
  }
}
