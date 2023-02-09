// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EndEffector;
import edu.wpi.first.wpilibj.PS4Controller;

public class EndEffectorCommand extends CommandBase {
  PS4Controller ps4 = new PS4Controller(1);
  /** Creates a new EndEffectorCommand. */
  public EndEffector endEffector;
  public EndEffectorCommand(EndEffector effector) {
    addRequirements(effector);
    this.endEffector = effector;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    endEffector.setEndEffectorBrakeMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    endEffector.setEndEffectorPower(0.5*ps4.getRawAxis(5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setEndEffectorPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}