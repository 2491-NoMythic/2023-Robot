package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.settings.Constants.PS4;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class BalanceCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final PIDController balanceController;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    public BalanceCommand(DrivetrainSubsystem drivetrainSubsystem,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrainSubsystem;

        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;

        balanceController = new PIDController(DriveConstants.k_BALANCE_P, DriveConstants.k_BALANCE_I, DriveConstants.k_BALANCE_D);
        balanceController.setTolerance(DriveConstants.k_BALANCE_TOLORANCE_DEGREES, DriveConstants.k_BALANCE_TOLORANCE_DEG_PER_SEC);
        
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        drivetrain.drive(new ChassisSpeeds(
                balanceController.calculate(drivetrain.getGyroscopePitch().getDegrees(), 0) * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
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