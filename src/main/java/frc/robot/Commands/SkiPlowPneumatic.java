// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SkiPlow;

public class SkiPlowPneumatic extends CommandBase {
  /** Creates a new SkiPlowPneumatic. */
  public SkiPlow skiplow;
  public PS4Controller opController;
  public double maxSpeed;
  public BooleanSupplier skiPlowDown;
  public BooleanSupplier lock;
  public BooleanSupplier rollCone;
  public BooleanSupplier rollCube;

  public SkiPlowPneumatic(SkiPlow skiPlow, BooleanSupplier SkiPlowDown, BooleanSupplier PneumaticLock, BooleanSupplier RollerCone, BooleanSupplier RollerCube, double maxSpeed) {
    this.skiplow =  skiPlow;
    this.opController = opController;
    this.maxSpeed = maxSpeed;

    addRequirements(skiPlow);

    skiPlowDown = SkiPlowDown;
    lock = PneumaticLock;
    rollCone = RollerCone;
    rollCube  = RollerCube;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(opController.getCrossButton()) {
    //   skiplow.pistonDown();
    // }
    // if(opController.getCircleButton()) {
    //   skiplow.pistonUp();
    // }
    // if(opController.getSquareButton()) {
    //   skiplow.pistonDown();
    // }
    // if(opController.getTriangleButton()) {
    //   skiplow.pistonUp();
    // }
    // if(opController.getR2Button()) {
    //   skiplow.lockOn();}
    // else{skiplow.lockOff();} 
    if(skiPlowDown.getAsBoolean()) {
     skiplow.pistonDown();}
    else skiplow.pistonUp();

    if(lock.getAsBoolean()) {
     skiplow.lockOn();}
    else skiplow.lockOff(); 

    if (rollCone.getAsBoolean()) {
      skiplow.rollerCone(maxSpeed);
    } else if(rollCube.getAsBoolean()){
        skiplow.rollerCube(-maxSpeed);
    } else {
      skiplow.rollerOff();
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
