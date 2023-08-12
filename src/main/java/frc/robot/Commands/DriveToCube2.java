// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class DriveToCube2 extends CommandBase {
  private final DrivetrainSubsystem drivetrain;
  private final Limelight ll;

  private LimelightDetectorData detectorData;
  private double tx;
  private double ty;
  private PIDController txController;
  private PIDController tyController;

  /** Creates a new DrivePickupCube. */
  public DriveToCube2(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    this.ll = Limelight.getInstance();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    txController = new PIDController(
        DriveConstants.K_DETECTOR_TX_P,
        DriveConstants.K_DETECTOR_TX_I,
        DriveConstants.K_DETECTOR_TX_D);
    tyController = new PIDController(
        DriveConstants.K_DETECTOR_TY_P,
        DriveConstants.K_DETECTOR_TY_I,
        DriveConstants.K_DETECTOR_TY_D);

    txController.setSetpoint(0);
    tyController.setSetpoint(0);
    txController.setTolerance(1);
    tyController.setTolerance(1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    detectorData = Limelight.latestDetectorValues;
    if (detectorData == null) {
      drivetrain.stop();
      System.err.println("nullDetectorData");
      return;
    }
    if (!detectorData.isResultValid) {
      drivetrain.stop();
      System.err.println("invalidDetectorData");
      return;
    }
    
    tx = detectorData.tx;
    ty = detectorData.ty;
    
    SmartDashboard.putNumber("Ttx", txController.calculate(tx));
    SmartDashboard.putNumber("Tty", tyController.calculate(ty));

    if (tyController.atSetpoint() && txController.atSetpoint()) {
      drivetrain.stop();
    } else {
      drivetrain.drive(new ChassisSpeeds(tyController.calculate(ty), 0, txController.calculate(tx)));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (tyController.atSetpoint() && txController.atSetpoint());
  }
}
