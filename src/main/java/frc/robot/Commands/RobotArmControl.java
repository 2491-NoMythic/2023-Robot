// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotArmSubsystem;


public class RobotArmControl extends CommandBase {
  private RobotArmSubsystem arm;
  private PS4Controller ps4 = new PS4Controller(1);
  /** Creates a new RobotArm. */
  double shoulderSpeed;
  double elbowSpeed;
  public RobotArmControl(RobotArmSubsystem Arm) {
    addRequirements(Arm);
    arm = Arm;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elbowSpeed = ps4.getRawAxis(5);
    shoulderSpeed = ps4.getRawAxis(1);
    arm.setElbowPower(elbowSpeed);
    arm.setShoulderPower(shoulderSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.setElbowPower(0);
    arm.setShoulderPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
