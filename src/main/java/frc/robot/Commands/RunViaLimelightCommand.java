// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightmotorSubsystem;
import frc.robot.settings.LimelightValues;
public class RunViaLimelightCommand extends CommandBase {
  /** Creates a new RunViaLimelightCommand. */
  private LimelightmotorSubsystem limelightMotor;
  Limelight limelight;

  public RunViaLimelightCommand(LimelightmotorSubsystem limelightMotor) {
    limelight = Limelight.getInstance();
    addRequirements(limelightMotor);
    this.limelightMotor = limelightMotor;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //need to add stuff
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    LimelightValues limelightValues = limelight.getLimelightValues();

    limelightMotor.runLimelightmotor(limelightValues.gettx()*0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {// empty for a reason
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
