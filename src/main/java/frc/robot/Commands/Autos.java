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
        // autoChooser.addOption("intakeDown", intakeDown());
        autoChooser.addOption("N2score2balance", N2Score2Bal());
        autoChooser.addOption("N2score2", N2Score2());
        autoChooser.addOption("N8score2balance", N8Score2Bal());
        autoChooser.addOption("N8score2", N8Score2());
        autoChooser.addOption("score1", score1());
        autoChooser.addOption("score1TaxiBalance", score1TaxiBal());
        autoChooser.addOption("score1balance", score1Bal());
        autoChooser.addOption("score1Taxi", score1Taxi());
        autoChooser.addOption("forward180", forward180());
        autoChooser.addOption("coolCircle", coolCircle());
    }

    public CommandBase moveToPose(Pose2d targetPose) {
        return Commands.runOnce(()->{
            Pose2d currentPose = drivetrain.getPose();
            Rotation2d heading = (targetPose.getTranslation().minus(currentPose.getTranslation())).getAngle();
            PathPlannerTrajectory newTraj = PathPlanner.generatePath(
                    new PathConstraints(1, 1),
                    new PathPoint(currentPose.getTranslation(), heading, currentPose.getRotation()),
                    new PathPoint(targetPose.getTranslation(), heading, targetPose.getRotation()));
            drivetrain.displayFieldTrajectory(newTraj);
            // SmartDashboard.putString("targetPose", targetPose.toString());
            autoBuilder.followPath(newTraj).finallyDo(x -> drivetrain.stop()).schedule();
        });
        
    }
    public CommandBase moveToNearestNode() {
        return moveToPose(drivetrain.getNearestNode());
    }
    // public SequentialCommandGroup intakeDown() {
        // return new SequentialCommandGroup(eventMap.get("IntakeDown"), eventMap.get("IntakeUp"));
    // }
    public SequentialCommandGroup N2Score2Bal() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(N2Score2Bal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N2Score2() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(N2Score2),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N8Score2Bal() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(N8Score2Bal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup N8Score2() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(N8Score2),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain)); 
    }

    public SequentialCommandGroup score1() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(Score1),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup score1Bal() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(Score1Bal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup score1TaxiBal() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(Score1TaxiBal),
            new DriveBalanceCommand(drivetrain),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup score1Taxi() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(Score1Taxi),
            new InstantCommand(drivetrain::pointWheelsInward, drivetrain));
    }
    public SequentialCommandGroup forward180() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(forward180Path));
    }
    public SequentialCommandGroup coolCircle() {
        return new SequentialCommandGroup(
            new InstantCommand(drivetrain::zeroGyroscope, drivetrain),
            autoBuilder.fullAuto(coolCirclePath));
    }
    // load all paths.
    static List<PathPlannerTrajectory> N2Score2Bal = PathPlanner.loadPathGroup("N2Score2Bal", new PathConstraints(4, 2.5));
    static List<PathPlannerTrajectory> N2Score2 = PathPlanner.loadPathGroup("N2Score2", new PathConstraints(4, 2.5));

    static List<PathPlannerTrajectory> N8Score2Bal = PathPlanner.loadPathGroup("N8Score2Bal", new PathConstraints(4, 2.5));
    static List<PathPlannerTrajectory> N8Score2 = PathPlanner.loadPathGroup("N8Score2", new PathConstraints(4, 1.75));
    static List<PathPlannerTrajectory> Score1 = PathPlanner.loadPathGroup("Score1", new PathConstraints(1, 1));

    static List<PathPlannerTrajectory> Score1Taxi = PathPlanner.loadPathGroup("Score1Taxi", new PathConstraints(2.5, 1.5));
    static List<PathPlannerTrajectory> Score1Bal = PathPlanner.loadPathGroup("Score1Bal", new PathConstraints(2.5, 1.5));
    static List<PathPlannerTrajectory> Score1TaxiBal = PathPlanner.loadPathGroup("Score1TaxiBal", new PathConstraints(2.5, 1.5));

    static List<PathPlannerTrajectory> forward180Path = PathPlanner.loadPathGroup("forward 180", new PathConstraints(3, 1.5));
    static List<PathPlannerTrajectory> coolCirclePath = PathPlanner.loadPathGroup("cool circle", new PathConstraints(3, 1.5));
}
