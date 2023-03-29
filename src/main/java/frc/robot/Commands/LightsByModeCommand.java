package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemLights;

/**
 * Run lights based on the GamePieceMode.
 */
public class LightsByModeCommand extends CommandBase {
  private SubsystemLights subsystemLights;
  private GamePieceMode gamePieceMode;
  private boolean blink;
  private boolean isBlinking = false;
  private int counter = 0;
  private static int RATE = 15;

  public enum GamePieceMode {
    CubeMode1,
    ConeMode1,
    ConeMode2,
    ConeMode3;
  }

  /**
   * Create light command based on the GamePieceMode.
   * Lights will blink at constant rate if blink is true
   * 
   * @param subsystemLights Subsystem reference
   * @param gamePieceMode determines the lights shown
   * @param blink lights if true
   */
  public LightsByModeCommand(SubsystemLights subsystemLights, GamePieceMode gamePieceMode, boolean blink) {
    addRequirements(subsystemLights);
    this.subsystemLights = subsystemLights;
    this.gamePieceMode = gamePieceMode;
    this.blink = blink;
  }

  private void drawCubeMode1() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 51, 0, 0, 100);
    subsystemLights.dataSetter();
  }

  private void drawConeMode1() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 51, 100, 64, 0);
    subsystemLights.dataSetter();
  }

  private void drawConeMode2() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 8, 100, 64, 100);
    subsystemLights.setLights(16, 32, 100, 64, 100);
    subsystemLights.setLights(40, 48, 100, 64, 100);
    subsystemLights.dataSetter();
  }

  private void drawConeMode3() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(8, 16, 100, 64, 100);
    subsystemLights.setLights(32, 40, 100, 64, 100);
    subsystemLights.dataSetter();
  }

  private void drawConeMode() {
    switch(this.gamePieceMode) {
      case ConeMode1: 
        drawConeMode1();
        break;
      case ConeMode2: 
        drawConeMode2();
        break;
      case ConeMode3: 
        drawConeMode3();
        break;
      default: drawCubeMode1();
    }
    subsystemLights.dataSetter();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
    drawConeMode();
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * After being called RATE times, it will blink if blink is true
   */
  @Override
  public void execute() {
    if (blink) {
      counter++;
      if (counter > RATE) {
        if (isBlinking) {
          subsystemLights.lightsOut();
          subsystemLights.dataSetter();
        } else {
          drawConeMode();
        }
        counter = 0;
        isBlinking = !isBlinking;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    subsystemLights.lightsOut();
    subsystemLights.dataSetter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}