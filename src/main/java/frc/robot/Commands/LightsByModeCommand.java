package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.settings.IntakeState;
import frc.robot.subsystems.SubsystemLights;

/**
 * Run lights based on the GamePieceMode.
 */
public class LightsByModeCommand extends CommandBase {
  private SubsystemLights subsystemLights;
  public IntakeState intakeState;
  private boolean blink;
  private boolean isBlinking = false;
  private int counter = 0;
  private static int RATE = 15;


  /**
   * Create light command based on the GamePieceMode.
   * Lights will blink at constant rate if blink is true
   * 
   * @param subsystemLights Subsystem reference
   * @param gamePieceMode determines the lights shown
   * @param blink lights if true
   */
  public LightsByModeCommand(SubsystemLights subsystemLights, boolean blink) {
    addRequirements(subsystemLights);
    this.subsystemLights = subsystemLights;
    this.blink = blink;
    this.intakeState = IntakeState.getInstance();
  }

  private void drawCubeModeGround() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 51, 50, 0, 100);
    subsystemLights.dataSetter();
  }
  private void drawCubeModeRamp() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(8, 16, 50, 0, 100);
    subsystemLights.setLights(32, 40, 50, 0, 100);
    subsystemLights.dataSetter();
  }
  private void drawCubeModeShelf() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 8, 50, 0, 100);
    subsystemLights.setLights(16, 32, 50, 0, 100);
    subsystemLights.setLights(40, 48, 50, 0, 100);
    subsystemLights.dataSetter();
  }
  private void drawConeModeGround() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 51, 150, 100, 0);
    subsystemLights.dataSetter();
  }
  private void drawConeModeRamp() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(8, 16, 150, 100, 0);
    subsystemLights.setLights(32, 40, 150, 100, 0);
    subsystemLights.dataSetter();
  }
  private void drawConeModeShelf() {
    subsystemLights.lightsOut();
    subsystemLights.setLights(0, 8, 150, 100, 0);
    subsystemLights.setLights(16, 32, 150, 100, 0);
    subsystemLights.setLights(40, 48, 150, 100, 0);
    subsystemLights.dataSetter();
  }


  private void drawConeMode() {
    switch(intakeState.getIntakeMode()) {

      case CONE_GROUND:
        drawConeModeGround();
        break;

      case CONE_RAMP:
        drawConeModeRamp();
        break;

      case CONE_SHELF:
        drawConeModeShelf();
        break;

      case CUBE_RAMP:
        drawCubeModeRamp();
        break;

      case CUBE_SHELF:
        drawCubeModeShelf();
        break;

      default:
        drawCubeModeGround();
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
    else drawConeMode();
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
