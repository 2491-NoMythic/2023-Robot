// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemLights;
import java.util.Timer;

public class PurpleLights extends CommandBase {
  Timer timer;
  SubsystemLights lights;
  int purple1;
  int purple2;
  int purple3;
  int purple4;

  /** Creates a new purpleLights. */
  public PurpleLights(SubsystemLights LS) {
    addRequirements(LS);
    lights = LS;
    timer = new Timer();
    purple1 = 0;
    purple2 = 1;
    purple3 = 2;
    purple4 = 3;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lights.setLights(0, 52, 100, 50, 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    lights.setLights(0, 52, 150, 140, 150);
    int pr = 50, pg = 0, pb = 80;
    for(int i = purple1; i < purple1+8; i++) {
      lights.setOneLightRGB((i)%52, pr, pg, pb);
      lights.setOneLightRGB((i+18)%52, pr, pg, pb);
      lights.setOneLightRGB((i+36)%52, pr, pg, pb);
    }
    lights.dataSetter();
    purple1++;
    purple4++;
   
    if(purple1>51) {
      purple1 = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.lightsOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
