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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Limelight;

public final class Autos {
    private DrivetrainSubsystem drivetrain;
    public static SwerveAutoBuilder autoBuilder;

    private static Autos autos;

    private Autos() {
    }

    public static Autos getInstance() {
        if (autos == null) {
            autos = new Autos();
        }
        return autos;
    }

    public void autoInit(SendableChooser<Command> autoChooser, HashMap<String, Command> eventMap, DrivetrainSubsystem drivetrain) {
        this.drivetrain = drivetrain;
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
        // autoChooser.addOption("N1HighN3HighMid", N1HighN3HighMid());
        // autoChooser.addOption("N9HighN8HighMid", N9HighN8HighMid());
        // autoChooser.addOption("N4HighBal", N4HighBal());
        // autoChooser.addOption("N4HighN5HighBal", N4HighN5HighBal());
        autoChooser.addOption("N1MidN3MidHigh", N1MidN3MidHigh());
        autoChooser.addOption("N9MidN8MidHigh", N9MidN8MidHigh());
        autoChooser.addOption("N7HighN8MidHigh", N7HighN8MidHigh());
        autoChooser.addOption("N4MidBal", N4MidBal());
        autoChooser.addOption("N4MidN5MidBal", N4MidN5MidBal());
        autoChooser.addOption("N4MidN5MidBalLL", N4HighGrabBal());
        autoChooser.addOption("driveForward", driveForward());
        autoChooser.addOption("highConeDriveForward", highConeDriveForward());
    }

    public CommandBase moveToPose(Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getPose();
        if (currentPose.getTranslation().getDistance(targetPose.getTranslation())<1){
            return Commands.runOnce(()->{
                Rotation2d heading = (targetPose.getTranslation().minus(currentPose.getTranslation())).getAngle();
                PathPlannerTrajectory newTraj = PathPlanner.generatePath(
                        new PathConstraints(1, 1),
                        new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
                        new PathPoint(targetPose.getTranslation(), heading, targetPose.getRotation()));
                drivetrain.displayFieldTrajectory(newTraj);
                autoBuilder.followPath(newTraj).withTimeout(newTraj.getTotalTimeSeconds()).finallyDo(x -> drivetrain.stop()).schedule();
            });
        } else {return new InstantCommand();}
    }
    public PathPlannerTrajectory calculatePath(Pose2d currentPose, Pose2d targetPose) {
        Rotation2d heading = (targetPose.getTranslation().minus(currentPose.getTranslation())).getAngle();
        PathPlannerTrajectory trajectory = PathPlanner.generatePath(
                new PathConstraints(1, 1),
                new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
                new PathPoint(targetPose.getTranslation(), heading, targetPose.getRotation()));
        drivetrain.displayFieldTrajectory(trajectory);
        return trajectory;
    }
    public CommandBase followPath(PathPlannerTrajectory path) {
        return autoBuilder.followPath(path);
    }
    public CommandBase moveToNearestNode() {
        return moveToPose(drivetrain.getNearestNode());
    }
    // public SequentialCommandGroup N4HighBal() {
    //     return new SequentialCommandGroup(
    //         autoBuilder.fullAuto(N4HighBal),
    //         new DriveBalanceCommand(drivetrain),
    //         new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    // }
    // public SequentialCommandGroup N4HighN5HighBal() {
    //     return new SequentialCommandGroup(
    //         autoBuilder.fullAuto(N4HighN5HighBal),
    //         new DriveBalanceCommand(drivetrain),
    //         new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    // }

    // public SequentialCommandGroup N1HighN3HighMid() {
    //     return new SequentialCommandGroup(
    //         autoBuilder.fullAuto(N1HighN3HighMid),
    //         new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    // }
    // public SequentialCommandGroup N9HighN8HighMid() {
    //     return new SequentialCommandGroup(
    //         autoBuilder.fullAuto(N9HighN8HighMid),
    //         new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    // }
    public SequentialCommandGroup N4MidBal() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N4MidBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N4MidN5MidBal() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N4MidN5MidBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N4HighGrabBal() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N4HighGrabBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }

    public SequentialCommandGroup N1MidN3MidHigh() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N1HighN3MidHigh),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N9MidN8MidHigh() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N9MidN8MidHigh),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N7HighN8MidHigh() {
        return new SequentialCommandGroup(
            autoBuilder.fullAuto(N7HighN8MidHigh),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup driveForward() {
        return new SequentialCommandGroup(
            new InstantCommand(()->Limelight.useAprilTagLimelight(false)),
            autoBuilder.fullAuto(driveForward),
            new InstantCommand(()->Limelight.useAprilTagLimelight(true))
            );
    }
    public SequentialCommandGroup highConeDriveForward() {
        return new SequentialCommandGroup(
            new InstantCommand(()->Limelight.useAprilTagLimelight(false)),
            autoBuilder.fullAuto(highConeDriveForward),
            new InstantCommand(()->Limelight.useAprilTagLimelight(true))
            );
    }

    // load all paths.

    // static List<PathPlannerTrajectory> N1HighN3HighMid = PathPlanner.loadPathGroup("N1HighN3HighMid", new PathConstraints(2.5, 1.75));
    // static List<PathPlannerTrajectory> N9HighN8HighMid = PathPlanner.loadPathGroup("N9HighN8HighMid", new PathConstraints(3, 2.5));
    // static List<PathPlannerTrajectory> N4HighBal = PathPlanner.loadPathGroup("N4HighBal", new PathConstraints(2.5, 1.75));
    // static List<PathPlannerTrajectory> N4HighN5HighBal = PathPlanner.loadPathGroup("N4HighN5HighBal", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> driveForward = PathPlanner.loadPathGroup("driveForward", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> highConeDriveForward = PathPlanner.loadPathGroup("highConeDriveForward", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N1HighN3MidHigh = PathPlanner.loadPathGroup("IRIS N1HighN3MidHigh", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N9MidN8MidHigh = PathPlanner.loadPathGroup("IRIS N9MidN8MidHigh", new PathConstraints(3, 2.5));
    static List<PathPlannerTrajectory> N7HighN8MidHigh = PathPlanner.loadPathGroup("IRIS N7HighN8MidHigh", new PathConstraints(3, 2.5));
    static List<PathPlannerTrajectory> N4MidBal = PathPlanner.loadPathGroup("IRIS N4MidBal", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N4MidN5MidBal = PathPlanner.loadPathGroup("IRIS N4MidN5MidBal", new PathConstraints(2.5, 1.75));
    static List<PathPlannerTrajectory> N4HighGrabBal = PathPlanner.loadPathGroup("IRIS N4HighGrabBal", new PathConstraints(2.5, 1.75));

}
