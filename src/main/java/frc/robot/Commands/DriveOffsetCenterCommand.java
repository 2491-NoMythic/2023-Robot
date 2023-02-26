package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveOffsetCenterCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrain;
    private final BooleanSupplier robotCentricMode;
    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;
    private ChassisSpeeds chassisSpeeds;
    private SwerveModuleState[] desiredStates;
    private final SwerveDriveKinematics kinematics;
    private final Translation2d offset;
    /**This is a temporary command to experiment with offset drive centers. Probably shouldn't merge into main.
     * @param offset from the center of the robot in meters. X= -.45 should be around in between the ski plows.
    */
    public DriveOffsetCenterCommand(DrivetrainSubsystem drivetrainSubsystem,
    BooleanSupplier robotCentricMode,
    DoubleSupplier translationXSupplier,
    DoubleSupplier translationYSupplier,
    DoubleSupplier rotationSupplier, 
    Translation2d offset) {
        this.drivetrain = drivetrainSubsystem;
        this.robotCentricMode = robotCentricMode;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.offset = offset;
        kinematics = DriveConstants.kinematics;
        addRequirements(drivetrainSubsystem);
    }
    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        if (robotCentricMode.getAsBoolean()) {
            chassisSpeeds = new ChassisSpeeds(
                translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
            
            desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds, offset);

            drivetrain.setModuleStates(desiredStates);
        
        } else {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                translationXSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                translationYSupplier.getAsDouble() * DriveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                rotationSupplier.getAsDouble() * DriveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                drivetrain.getGyroscopeRotation());

            desiredStates = kinematics.toSwerveModuleStates(chassisSpeeds, offset);
        
            drivetrain.setModuleStates(desiredStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}