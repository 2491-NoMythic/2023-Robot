// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkiPlow;

public class SkiPlowPneumatic extends CommandBase {
  /** Creates a new SkiPlowPneumatic. */
  public SkiPlow skiplow;
  public PS4Controller opController;
  public SkiPlowPneumatic(SkiPlow skiPlow, PS4Controller opController) {
    this.skiplow =  skiPlow;
    this.opController = opController;
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
    if(opController.getL2Button()) {
      skiplow.pistonDownLeft();
    }else{skiplow.pistonUpLeft();}

    if(opController.getR2Button()) {
      skiplow.pistonDownRight();}
    else{skiplow.pistonUpRight();}

    if(opController.getCrossButton()) {
      skiplow.lockOn();}
    else{skiplow.lockOff();} 
      
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
