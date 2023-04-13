package frc.robot.Commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class Autos {
    private DrivetrainSubsystem drivetrain;
    private static SwerveAutoBuilder autoBuilder;
    private HashMap<String, Command> eventMap;

    private static Autos autos;

    private void Autos() {
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

    public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
        this.eventMap = eventMap;
        // Create the AutoBuilder. This only needs to be created once when robot code
        // starts, not every time you want to create an auto command. A good place to
        // put this is in RobotContainer along with your subsystems.
        Autos.autoBuilder = new SwerveAutoBuilder(
                drivetrain::getPose, // Pose2d supplier
                drivetrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
                drivetrain.kinematics, // SwerveDriveKinematics
                new PIDConstants(
                        DriveConstants.k_XY_P,
                        DriveConstants.k_XY_I,
                        DriveConstants.k_XY_D), // PID constants to correct for translation error (used to create the X
                                                // and Y PID controllers)
                new PIDConstants(
                        DriveConstants.k_THETA_P,
                        DriveConstants.k_THETA_I,
                        DriveConstants.k_THETA_D), // PID constants to correct for rotation error (used to create the
                                                   // rotation controller)
                drivetrain::setModuleStates, // Module states consumer used to output to the drive subsystem
                eventMap,
                true, // Should the path be automatically mirrored depending on alliance color.
                       // Optional, defaults to true
                drivetrain // The drive subsystem. Used to properly set the requirements of path following
                           // commands
        );
        // add autos to smart dashboard.\
        autoChooser.addOption("N1HighN3HighMid", N1HighN3HighMid());
        autoChooser.addOption("N9HighN8HighMid", N9HighN8HighMid());
        autoChooser.addOption("N4HighBal", N4HighBal());
        autoChooser.addOption("N4HighN5HighBal", N4HighN5HighBal());

    }

    public CommandBase moveToPose(Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getPose();
        if (currentPose.getTranslation().getDistance(targetPose.getTranslation())<3){
            return Commands.runOnce(()->{
                Rotation2d heading = (targetPose.getTranslation().minus(currentPose.getTranslation())).getAngle();
                PathPlannerTrajectory newTraj = PathPlanner.generatePath(
                        new PathConstraints(1, 1),
                        new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
                        new PathPoint(targetPose.getTranslation(), heading, targetPose.getRotation()));
                drivetrain.displayFieldTrajectory(newTraj);
                autoBuilder.followPath(newTraj).finallyDo(x -> drivetrain.stop()).schedule();
            });
        } else {return new InstantCommand();}
    }
    public CommandBase moveToNearestNode() {
        return moveToPose(drivetrain.getNearestNode());
    }
    public SequentialCommandGroup N4HighBal() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N4HighBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N4HighN5HighBal() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N4HighN5HighBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }

    public SequentialCommandGroup N1HighN3HighMid() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N1HighN3HighMid),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N9HighN8HighMid() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N9HighN8HighMid),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }

    // load all paths.

    static List<PathPlannerTrajectory> N1HighN3HighMid = PathPlanner.loadPathGroup("N1HighN3HighMid", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N9HighN8HighMid = PathPlanner.loadPathGroup("N9HighN8HighMid", new PathConstraints(3, 2.5));
    static List<PathPlannerTrajectory> N4HighBal = PathPlanner.loadPathGroup("N4HighBal", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N4HighN5HighBal = PathPlanner.loadPathGroup("N4HighN5HighBal", new PathConstraints(2.5, 1.75));

}
