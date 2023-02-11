// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;
import edu.wpi.first.wpilibj.PS4Controller;

public class LightsSquareTriangle extends CommandBase {
  /** Creates a new LightsSquareTriangle. */
  Lights LS;
  PS4Controller ps4;
  public LightsSquareTriangle(Lights lights) {
    addRequirements(lights);
    LS = lights;
    ps4 = new PS4Controller(1);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LS.setLights(30, 30, 30);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ps4.getTriangleButton()) {
      LS.lightsOut();
      LS.setCertainLights(29, 59, 200, 30, 30);
      LS.dataSetter();
    }
    if(ps4.getSquareButton()) {
      LS.lightsOut();
      LS.setCertainLights(0, 39, 0, 0, 100);
      LS.dataSetter();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
