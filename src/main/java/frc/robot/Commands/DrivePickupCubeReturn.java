// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.arm.IntakeCube;
import frc.robot.Commands.arm.ResetFast;
import frc.robot.settings.IntakeState;
import frc.robot.settings.IntakeState.IntakeMode;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DrivePickupCubeReturn extends SequentialCommandGroup {

  private SwerveAutoBuilder autoBuilder = Autos.autoBuilder;
  private List<PathPlannerTrajectory> fullAuto = PathPlanner.loadPathGroup("VisionGrabCube", new PathConstraints(2.5, 1.25));
  
  /** Creates a new DrivePickupCubeReturn. */
  public DrivePickupCubeReturn(DrivetrainSubsystem drivetrain, ArmSubsystem arm, EndEffector effector) {
    addCommands(
      autoBuilder.resetPose(fullAuto.get(0)),
      autoBuilder.followPath(fullAuto.get(0)),
      new InstantCommand(()->IntakeState.setIntakeMode(IntakeMode.CUBE_GROUND)),
      new DriveToCube2(drivetrain),
      new EndEffectorRun(effector, ()->true),
      new IntakeCube(arm),
      new DriveForSeconds(drivetrain, new ChassisSpeeds(-.8, 0, 0), 1),
      new InstantCommand(effector::rollerOff, effector),
      new ResetFast(arm),
      autoBuilder.followPath(fullAuto.get(1)),
      new InstantCommand(drivetrain::stop, drivetrain));
  }
}
