package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DoNothingCommand extends CommandBase {
    
  private double endTimeInMillis;
  private final double secondsToWait;

  /**
   * Creates a new DoNothingCommand. This command will do nothing, and end after the specified duration.
   * Should run like any command and not interrupt other running commands.
   * Uses System.currentTimeMillis()
   *
   * @param seconds the time to wait, in seconds
   */
  public DoNothingCommand(double seconds) {
    secondsToWait = seconds;
  }

  @Override
  public void initialize() {
    endTimeInMillis = System.currentTimeMillis() + secondsToWait * 1000;
  }

  @Override
  public boolean isFinished() {
    return System.currentTimeMillis() > endTimeInMillis;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

}
