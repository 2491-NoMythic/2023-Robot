// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RobotArmSubsystem;


public class RobotArmControl extends CommandBase {
  private RobotArmSubsystem arm;
  /** Creates a new RobotArm. */
  double shoulderSpeed;
  double elbowSpeed;
  DoubleSupplier shoulderAxisSupplier;
  DoubleSupplier elbowAxisSupplier;

  public RobotArmControl(RobotArmSubsystem Arm,
    DoubleSupplier shoulderAxisSupplier,
    DoubleSupplier elbowAxisSupplier
  ) {
    this.shoulderAxisSupplier = shoulderAxisSupplier;
    this.elbowAxisSupplier = elbowAxisSupplier;
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
    elbowSpeed = elbowAxisSupplier.getAsDouble();
    shoulderSpeed = shoulderAxisSupplier.getAsDouble();
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
