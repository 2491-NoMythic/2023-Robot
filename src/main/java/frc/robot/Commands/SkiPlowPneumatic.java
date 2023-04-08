// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.SkiPlow;

public class SkiPlowPneumatic extends CommandBase {
  /** Creates a new SkiPlowPneumatic. */
  public SkiPlow skiplow;

  // public PS4Controller opController;
  public double maxSpeed;
  public BooleanSupplier skiPlowDown;
  // public BooleanSupplier lock;
  public IntakeState intakeState;

  public SkiPlowPneumatic(SkiPlow skiPlow, BooleanSupplier SkiPlowDown, double maxSpeed) {
    this.skiplow =  skiPlow;
    // this.opController = opController;

    this.maxSpeed = maxSpeed;

    addRequirements(skiPlow);

    skiPlowDown = SkiPlowDown;
    this.intakeState = intakeState.getInstance();
    // lock = PneumaticLock;

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

    // if(lock.getAsBoolean()) {
    //  skiplow.lockOn();}
    // else skiplow.lockOff(); 


    switch(intakeState.getIntakeMode()){
    case CUBE:{
      skiplow.rollerCube();
      break;
    }
    case CONE_GROUND:{
      skiplow.rollerCone();
      break;
    }
    case CONE_RAMP:{
      skiplow.rollerCone();
      break;
    }
    case CONE_SHELF:{
      skiplow.rollerCone();
      break;
    }
    default:{
      skiplow.rollerOff();
      break;
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
