// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class DriveToCube extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Limelight ll;

  private LimelightDetectorData detectorData;
  private double tx;
  private double ty;

  /** Creates a new DrivePickupCube. */
  public DriveToCube(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.ll = Limelight.getInstance();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override 
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectorData = ll.latestDetectorValues;
    if (detectorData == null) {
      drivetrain.stop();
      return;
    }
    if (!detectorData.isResultValid) {
      drivetrain.stop();
      return;
    }
    tx = detectorData.tx;
    ty = detectorData.ty;
    drivetrain.drive(new ChassisSpeeds(-ty/20, 0, -tx/20));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ty>=0 && ty<=2.5 && tx >=0 && tx<=1.5);
  }
}
