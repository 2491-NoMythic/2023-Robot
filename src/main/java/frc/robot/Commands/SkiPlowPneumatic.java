// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkiPlow;
import edu.wpi.first.wpilibj.PS4Controller;

public class SkiPlowPneumatic extends CommandBase {
  PS4Controller ps4 = new PS4Controller(0);
  /** Creates a new SkiPlowPneumatic. */
  public SkiPlow skiplow;
  public SkiPlowPneumatic(SkiPlow skiPlow) {
    this.skiplow =  skiPlow;
    addRequirements(skiPlow);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(ps4.getCrossButton()){
      skiplow.pistonForward();
    }
    else{
      if(ps4.getCircleButton()) {
        skiplow.pistonReverse();
      }
      else{
        skiplow.pistonoff();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
