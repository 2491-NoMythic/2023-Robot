package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveBalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PIDController balanceController;
    private double targetSpeedMetersPerSecond;
    private double timeBalanced;
    private double targetTime = 100;
    public DriveBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
        this.drivetrain = drivetrainSubsystem;

        balanceController = new PIDController(DriveConstants.k_BALANCE_P, DriveConstants.k_BALANCE_I, DriveConstants.k_BALANCE_D);
        balanceController.setTolerance(DriveConstants.k_BALANCE_TOLORANCE_DEGREES, DriveConstants.k_BALANCE_TOLORANCE_DEG_PER_SEC);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        targetSpeedMetersPerSecond = balanceController.calculate(drivetrain.getGyroscopePitch().getDegrees(), 0) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND/4;
        
        SmartDashboard.putNumber("pitch", drivetrain.getGyroscopePitch().getDegrees());
        SmartDashboard.putBoolean("IsAtTarget", balanceController.atSetpoint());
        
        if (balanceController.atSetpoint()) timeBalanced++; else timeBalanced = 0;
        
        drivetrain.drive(new ChassisSpeeds(
                targetSpeedMetersPerSecond,
                0, 0
            ));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
    public boolean isFinished() {
        return (timeBalanced >= targetTime);
    }
}