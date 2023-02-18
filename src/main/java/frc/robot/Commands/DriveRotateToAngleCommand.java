// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveRotateToAngleCommand extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier turnSupplier;
  private final DoubleSupplier rightStickMagnitudeSupplier;
  private final PIDController turnController;

  /** Creates a new DriveRotateToAngleCommand. */
  public DriveRotateToAngleCommand(DrivetrainSubsystem drivetrainSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier turnSupplier,
      DoubleSupplier rightStickMagnitudeSupplier) {
    this.drivetrain = drivetrainSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.turnSupplier = turnSupplier;
    this.rightStickMagnitudeSupplier = rightStickMagnitudeSupplier;

    turnController = new PIDController(DriveConstants.k_THETA_P, DriveConstants.k_THETA_I, DriveConstants.k_THETA_D);
    turnController.enableContinuousInput(-180.0, 180.0);
    turnController.setTolerance(DriveConstants.k_THETA_TOLORANCE_DEGREES, DriveConstants.k_THETA_TOLORANCE_DEG_PER_SEC);
    addRequirements(drivetrainSubsystem);
  }

  @Override
  public void execute() {
    drivetrain.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
        translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        rightStickMagnitudeSupplier.getAsDouble()
            * (MathUtil.clamp(
            turnController.calculate(drivetrain.getGyroscopeRotation().getDegrees(), turnSupplier.getAsDouble()), -1, 1)
            * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND),
        drivetrain.getGyroscopeRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  public boolean isFinished() {
    return false;
  }
}
