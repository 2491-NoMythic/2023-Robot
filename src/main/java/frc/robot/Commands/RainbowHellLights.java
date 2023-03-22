package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SubsystemLights;

public class RainbowHellLights extends CommandBase{
    SubsystemLights lights;
    private int m_rainbowFirstPixelHue;
    public RainbowHellLights(SubsystemLights LS) {
        addRequirements(LS);
        lights = LS;
} 
@Override
public void initialize() {
    lights.setHSVLights(0, 52, 0, 0, 0);
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
    // For every pixel
    for (var i = 0; i < 52; i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / 52)) % 180;
        // Set the value
        lights.setHSVLights(0, 52, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
      lights.dataSetter();
    }
// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  lights.lightsOut();
  lights.dataSetter();
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}
}

