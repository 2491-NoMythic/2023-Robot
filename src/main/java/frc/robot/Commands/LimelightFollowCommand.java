// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.LimelightValues;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public class LimelightFollowCommand extends CommandBase {
    private DrivetrainSubsystem drivetrain;
    private PIDController balanceController;
    private Limelight limelight = Limelight.getInstance();

    public LimelightFollowCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrain = drivetrainSubsystem;

        balanceController = new PIDController(0.05, 0, 0);
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        LimelightValues limelightValues = limelight.getLimelightValues();
        double limelightValue = limelightValues.gettx(0);
        drivetrain.drive(new ChassisSpeeds(0, balanceController.calculate(limelightValue, 0), 0 
            ));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
    public boolean isFinished() {
        return false;
    }
}
