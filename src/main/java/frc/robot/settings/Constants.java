// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import com.ctre.phoenixpro.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
  public final class Intake{
    private Intake() {}
    public static final int INTAKE_MOTOR_ID = 16;
  }
  public final class Arm{
    private Arm(){}
    public static final int END_EFFECTOR_SMALL_ROLLER_ID = 2;
    public static final int END_EFFECTOR_BIG_ROLLER_ID = 1;
    public static final int ARM_SHOULDER_MOTOR_ID = 18;
    public static final int ARM_ELBOW_MOTOR_ID = 17;
    public static final String ARM_SHUFFLEBOARD_TAB = "Arm";
    public static final int ARM_SHOULDER_LOCK_CHANNEL = 4;
    public static final int ARM_ELBOW_LOCK_CHANNEL = 3;
    public static final double ARM_SHOULDER_ENCODER_OFFSET_DEG = 0.0;
    public static final double ARM_ELBOW_ENCODER_OFFSET = 0.91666;//TODO find offsets
    public static final double ARM_SHOULDER_LENGTH_METERS = 0.9906;
    public static final double ARM_ELBOW_LENGTH_METERS = 0.7366;
    public static final double ARM_ELBOW_CENTER_OF_MASS_OFFSET_METERS = 0.2;
    public static final double ARM_SHOULDER_ALLOWABLE_ERROR_DEG = 0.2;
    public static final double ARM_ELBOW_ALLOWABLE_ERROR_DEG = 0.2;
    public static final double ARM_SHOULDER_K_P = 0;
    public static final double ARM_SHOULDER_K_I = 0;
    public static final double ARM_SHOULDER_K_D = 0;
    public static final double ARM_SHOULDER_FF_K_G = 0;
    public static final double ARM_ELBOW_K_P = 0;
    public static final double ARM_ELBOW_K_I = 0;
    public static final double ARM_ELBOW_K_D = 0;
    public static final double ARM_ELBOW_FF_K_G = 0;
    public static final double ARM_SHOULDER_MAXVEL_RPM = 7.5;
    public static final double ARM_SHOULDER_MAXACC_RPM = 1;
    public static final double ARM_ELBOW_MAXVEL_RPM = 7.5;
    public static final double ARM_ELBOW_MAXACC_RPM = 1;
  }
  public static final class Preferences{
  
  }
  public static final class DriveConstants {
    public enum Positions{
      FL(0),
      FR(0.25),
      BL(0.5),
      BR(0.75);

      private double rotation;

      Positions(double value) {
        rotation = value;

      }
      public double getValue() {
        return rotation;
      }
    }
    public enum Offsets{
      AUGIE(0.153818),
      BENH(0.153564),
      EVELYN(-0.111084),
      OMARIAHN(0.266846),
      PHOEBE(-0.245850),
      ROYCE(-0.003174),
      ROWAN(0.391602),
      QUINN(0.355713),
      LIAM(0),
      LEVI(0.333008); //May be wrong, who knows
      private double offset;
      Offsets(double value) {
        offset = value;
      }
      public Rotation2d getValue(Positions pos) {
        return Rotation2d.fromRotations(MathUtil.inputModulus(offset+pos.getValue(), -0.5, 0.5));
      }
    }
    private DriveConstants() {
    }
    public static final Pose2d DRIVE_ODOMETRY_ORIGIN = new Pose2d(5.0, 5.0, new Rotation2d());
    /**
     * The left-to-right distance between the drivetrain wheels
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.52705;
    /**
     * The front-to-back distance between the drivetrain wheels.
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705;

    /**
     * The diameter of the module's wheel in meters.
     */
    public static final double DRIVETRAIN_WHEEL_DIAMETER = 0.10033;

    /**
     * The overall drive reduction of the module. Multiplying motor rotations by
     * this value should result in wheel rotations.
     */
    public static final double DRIVETRAIN_DRIVE_REDUCTION = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);

    /**
     * Whether the drive motor should be counterclockwise or clockwise positive. 
     * If there is an odd number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_DRIVE_INVERTED = InvertedValue.Clockwise_Positive;

    /**
     * The overall steer reduction of the module. Multiplying motor rotations by
     * this value should result in wheel rotations.
     */
    public static final double DRIVETRAIN_STEER_REDUCTION = (14.0 / 50.0) * (10.0 / 60.0);

    /**
     * Whether the steer motor should be counterclockwise or clockwise positive. 
     * If there is an odd number of gear reductions this is typically clockwise-positive.
     */
    public static final InvertedValue DRIVETRAIN_STEER_INVERTED = InvertedValue.Clockwise_Positive;

    /**
     * How many meters the wheels travel per rotation. <p>
     * Multiply rotations by this to get meters.<p>
     * Divide meters by this to get rotations.
     */
    public static final double DRIVETRAIN_ROTATIONS_TO_METERS = (DRIVETRAIN_WHEEL_DIAMETER * Math.PI);

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight line.
     */
    /*
     * FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
     * The formula for calculating the theoretical maximum velocity is:
     * <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
     */
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        DRIVETRAIN_DRIVE_REDUCTION * DRIVETRAIN_WHEEL_DIAMETER * Math.PI;
    /**
     * The drive motor sensor value at a 100% duty cycle output in a straight line.
     */
    public static final double MAX_VELOCITY_RPS_EMPIRICAL = 15.697;
    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
        Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
        
        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    public static final String DRIVETRAIN_SMARTDASHBOARD_TAB = "Drivetrain";
    public static final String CANIVORE_DRIVETRAIN = "Swerve";
    public static final int DRIVETRAIN_PIGEON_ID = 0;

    public static final int FL_DRIVE_MOTOR_ID = 1;
    public static final int FL_STEER_MOTOR_ID = 2;
    public static final int FL_STEER_ENCODER_ID = 1;
    public static final Rotation2d FL_STEER_OFFSET = Rotation2d.fromRotations(0.272217);

    public static final int FR_DRIVE_MOTOR_ID = 3;
    public static final int FR_STEER_MOTOR_ID = 4;
    public static final int FR_STEER_ENCODER_ID = 2;
    public static final Rotation2d FR_STEER_OFFSET = Rotation2d.fromRotations(0.41333);

    public static final int BL_DRIVE_MOTOR_ID = 5;
    public static final int BL_STEER_MOTOR_ID = 6;
    public static final int BL_STEER_ENCODER_ID = 3;
    public static final Rotation2d BL_STEER_OFFSET = Rotation2d.fromRotations(-0.11792);

    public static final int BR_DRIVE_MOTOR_ID = 7;
    public static final int BR_STEER_MOTOR_ID = 8;
    public static final int BR_STEER_ENCODER_ID = 4;
    public static final Rotation2d BR_STEER_OFFSET = Rotation2d.fromRotations(0.403809);

    // Drive Motor
    public static final double k_DRIVE_P = 0.03;
    public static final double k_DRIVE_I = 0;
    public static final double k_DRIVE_D = 0;
    public static final double k_DRIVE_FF_S = 0;
    public static final double k_DRIVE_FF_V = 0;
    public static final double DRIVE_DEADBAND_MPS = 0.01;
    public static final double DRIVE_MOTOR_RAMP = 0.1;
    // Steer Motor
    /**
     * The maximum velocity of the steer motor. <p> 
     * This is the limit of how fast the wheels can rotate in place.
     */
    public static final double MAX_STEER_VELOCITY_RADIANS_PER_SECOND = Math.PI; // 1/2 rotation per second.
    /**
     * The maximum acceleration of the steer motor. <p>
     * This is the limit of how fast the wheels can change rotation speed.
     */
    public static final double MAX_STEER_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2 * Math.PI; 
    public static final double k_STEER_P = 8.0;
    public static final double k_STEER_I = 0.0;
    public static final double k_STEER_D = 0.0; 
    public static final double k_STEER_FF_S = 0.0;
    public static final double k_STEER_FF_V = 0.0;

    // Auto PID loops
    // twin pid controllers that control the x and y robot movements.
    public static final double k_XY_P = 2.5;
    public static final double k_XY_I = 0.0;
    public static final double k_XY_D = 0.0;

    public static final double k_THETA_P = 5.0;
    public static final double k_THETA_I = 0.0;
    public static final double k_THETA_D = 0.0;
    public static final double k_THETA_TOLORANCE_DEGREES = 2.0;
    public static final double k_THETA_TOLORANCE_DEG_PER_SEC = 10;

    public static final double k_BALANCE_P = 0.025;
    public static final double k_BALANCE_I = 0.0;
    public static final double k_BALANCE_D = 0.0;
    public static final double k_BALANCE_TOLORANCE_DEGREES = 10.0;
    public static final double k_BALANCE_TOLORANCE_DEG_PER_SEC = 1;
}
  public final class PS4Driver{
    private PS4Driver() {
    }
    public static final int CONTROLLER_ID = 1;
    /**Left stick Y-axis. <p> Left = -1 || Right = 1*/
    public static final int X_AXIS = 0; 
    /**Left stick X-axis. <p> Forwards = -1 || Backwards = 1*/
    public static final int Y_AXIS = 1;
    /**Right stick Z-axis. <p> Left = -1 || Right = 1*/
    public static final int Z_AXIS = 2;
    /**Right stick Z-rotate. <p> Forwards = -1 || Backwards = 1*/
    public static final int Z_ROTATE = 5;
    /**Value used to differentiate between angle 0 and rest position.*/
    public static final double NO_INPUT = 404;
    public static final double DEADBAND_NORMAL = 0.05;
    public static final double DEADBAND_LARGE = 0.1;
}
}
