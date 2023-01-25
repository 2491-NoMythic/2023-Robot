// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  private Constants () {}
  public static class OperatorConstants {

    private OperatorConstants () {}

    public static final int DRIVER_CONTROLLER_PORT = 0;
  }
  public final class Intake{
    private Intake() {
    }
    public static final int INTAKE_MOTOR_ID = 2491;
  }
  public final class Arm{
    private Arm(){

    }
    public static final int ARM_SHOULDER_MOTOR_ID = 2;
    public static final int ARM_ELBOW_MOTOR_ID = 1;
  }
  public final class PS4{
    public static final int CONTROLLER_ID = 1;
    public static final int LEFT_AXIS_ID = 1;
    public static final int RIGHT_AXIS_ID = 5;
}
}
