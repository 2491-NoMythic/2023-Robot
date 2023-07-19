// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenixpro.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    public static final int INTAKE_MOTOR_ID = 6;
  }
  public final class Arm{
    private Arm(){}
    public static final int END_EFFECTOR_SMALL_ROLLER_ID = 2;
    public static final int END_EFFECTOR_BIG_ROLLER_ID = 1;
    public static final double END_EFFECTOR_SMALL_PASSIVE_POWER = 0.09;
    public static final double END_EFFECTOR_BIG_PASSIVE_POWER = 0.125;
    public static final double END_EFFECTOR_SMALL_POWER = 0.3;
    public static final double END_EFFECTOR_BIG_POWER = 0.4;
    public static final double END_EFFECTOR_POWER_BONUS = 0.1;
    
    public static final int ARM_SHOULDER_MOTOR_ID = 18;
    public static final int ARM_ELBOW_MOTOR_ID = 17;
    public static final String ARM_SHUFFLEBOARD_TAB = "Arm";
    public static final int ARM_SHOULDER_LOCK_CHANNEL = 4;
    public static final int ARM_ELBOW_LOCK_CHANNEL = 3;
    // public static final double ARM_SHOULDER_ENCODER_OFFSET_DEG = 354.765;
    public static final double ARM_SHOULDER_ENCODER_OFFSET_DEG = 56.56;
    // public static final double ARM_SHOULDER_ENCODER_OFFSET_DEG = 131.39;
    // public static final double ARM_ELBOW_ENCODER_OFFSET = 26.318;
    // public static final double ARM_ELBOW_ENCODER_OFFSET = 167.1972;
    public static final double ARM_ELBOW_ENCODER_OFFSET = 138;
    public static final double ARM_SHOULDER_LENGTH_METERS = 0.9906;
    public static final double ARM_ELBOW_LENGTH_METERS = 0.7366;
    public static final double ARM_ELBOW_CENTER_OF_MASS_OFFSET_METERS = 0.7;
    public static final double ARM_SHOULDER_ALLOWABLE_ERROR_DEG = 1;
    public static final double ARM_ELBOW_ALLOWABLE_ERROR_DEG = 1.5;
    public static final double ARM_SHOULDER_LIMIT_FORWARD_DEG = 55;
    public static final double ARM_SHOULDER_LIMIT_REVERSE_DEG = 360-55; 
    public static final double ARM_ELBOW_LIMIT_FORWARD_DEG = 130;
    public static final double ARM_ELBOW_LIMIT_REVERSE_DEG = 360-90;
    public static final double ARM_SHOULDER_K_P = 0.035;
    public static final double ARM_SHOULDER_K_I = 0;
    public static final double ARM_SHOULDER_K_D = 0;
    public static final double ARM_SHOULDER_FF_K_G = -0.2;
    public static final double ARM_ELBOW_K_P = 0.02;
    public static final double ARM_ELBOW_K_I = 0;
    public static final double ARM_ELBOW_K_D = 0;
    // public static final double ARM_ELBOW_FF_K_G = 0.85;
    public static final double ARM_ELBOW_FF_K_G = 0.95;
    public static final double ARM_SHOULDER_MAXVEL_RPM = 7.5;
    public static final double ARM_SHOULDER_MAXACC_RPM = 1;
    public static final double ARM_ELBOW_MAXVEL_RPM = 7.5;
    public static final double ARM_ELBOW_MAXACC_RPM = 1;
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
      LEVI(-0.38501);
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
     * The bumper-to-bumper width of the robot.
     */
    public static final double DRIVETRAIN_ROBOT_WIDTH_METERS = 0.83;
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
    public static final double DEADBAND_NORMAL = 0.08;
    public static final double DEADBAND_LARGE = 0.1;
}
public static final class Vision {
  /** the position of the far left corner of the field in meters */
  public static final Translation2d FIELD_CORNER = new Translation2d(16.54, 8.02);
  
  public static final double POSE_DETECTION_DISTANCE_THRESHOLD = 0.5;
  public static final String LIMELIGHT_APRILTAG_NAME = "";
  public static final String LIMELIGHT_NEURAL_NAME = "limelightneural";
}
  public enum Poses{
    RESET(0,0, false),
    INTAKE_CONE(-33,5.5, true),
    INTAKE_CUBE(-40,9, true),
    DROP_LOW(8,33, false),
    MID_CONE(0,85, false),
    MID_CUBE(0,85, false),
    HIGH_CONE(29,110, false),
    HIGH_CUBE(29,110, false),
    SHELF_CONE(0,94, false),
    SHELF_CUBE(0,94, false),
    RAMP_CONE(7.75,-46, true),
    RAMP_CUBE(13,-50, true),
    AVOID_POST(-20, 45, true), //TODO: how far back to avoid?
    AVOID_BUMPER(-17.5,-4,true);
    private double[] angles = new double[2];
    private boolean requiresIntakeDown;
    private static final int SHOLDER = 0;
    private static final int ELBOW = 1;
    
    Poses(double sholder, double elbow, boolean requiresIntakeDown) {
    angles[SHOLDER] = sholder;
    angles[ELBOW] = elbow;
    this.requiresIntakeDown = requiresIntakeDown;
    }
    public double getSholderAngle() {return angles[SHOLDER];}
    public double getElbowAngle() {return angles[ELBOW];}

    /** sholder as a rotation 2d */
    public Rotation2d getSholder() {
      return Rotation2d.fromDegrees(angles[SHOLDER]);}

    /** elbow as a rotation 2d */
    public Rotation2d getElbow() {return Rotation2d.fromDegrees(angles[ELBOW]);}

    public Rotation2d[] getRotations() {
    return new Rotation2d[] {getSholder(),getElbow()};
    }
    public boolean isRequiresIntakeDown() {return requiresIntakeDown;};
  }
  public static final class nodePositions {
    public static final double ARM_OFFSET_FROM_CENTER = 0; //TODO
    public static final Pose2d BLUE1 = new Pose2d(1.85, 0.51+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180)); //*this nodoe is up against the wall so might be changed */
    public static final Pose2d BLUE2 = new Pose2d(1.85, 1.04+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE3 = new Pose2d(1.85, 1.63+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE4 = new Pose2d(1.85, 2.2+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE5 = new Pose2d(1.85, 2.75+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE6 = new Pose2d(1.85, 3.3+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE7 = new Pose2d(1.85, 3.87+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE8 = new Pose2d(1.85, 4.45+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));
    public static final Pose2d BLUE9 = new Pose2d(1.85, 4.98+ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(180));//*this node is up against the wall, we probably have to deliver out the back for this  */
    
    public static final Pose2d BLUE_RAMP = new Pose2d(14.25, 7.5,Rotation2d.fromDegrees(-90));
    public static final Pose2d BLUE_SHELF_LEFT = new Pose2d(15.75, 7.3,Rotation2d.fromDegrees(0));
    public static final Pose2d BLUE_SHELF_RIGHT = new Pose2d(15.75, 6.2,Rotation2d.fromDegrees(0));

    public static final Pose2d RED1 = new Pose2d(1.85, 7.51-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0)); //*this nodoe is up against the wall so might be changed */
    public static final Pose2d RED2 = new Pose2d(1.85, 6.98-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED3 = new Pose2d(1.85, 6.39-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED4 = new Pose2d(1.85, 5.82-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED5 = new Pose2d(1.85, 5.27-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED6 = new Pose2d(1.85, 4.72-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED7 = new Pose2d(1.85, 4.15-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED8 = new Pose2d(1.85, 3.57-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));
    public static final Pose2d RED9 = new Pose2d(1.85, 2.97-ARM_OFFSET_FROM_CENTER, Rotation2d.fromDegrees(0));//*this node is up against the wall, we probably have to deliver out the back for this  */

    public static final Pose2d RED_RAMP = new Pose2d(14.25, 0.52,Rotation2d.fromDegrees(-90));
    public static final Pose2d RED_SHELF_LEFT = new Pose2d(15.75, 0.72,Rotation2d.fromDegrees(0));
    public static final Pose2d RED_SHELF_RIGHT = new Pose2d(15.75, 1.82,Rotation2d.fromDegrees(0));

    private static final Pose2d[] nodeArray = new Pose2d[] {
        BLUE1, BLUE2, BLUE3, BLUE4, BLUE5, BLUE6, BLUE7, BLUE8, BLUE9,
        BLUE_RAMP, BLUE_SHELF_LEFT, BLUE_SHELF_RIGHT,
        RED1, RED2, RED3, RED4, RED5, RED6, RED7, RED8, RED9, RED_RAMP,
        RED_SHELF_LEFT, RED_SHELF_RIGHT };

    private static final Pose2d[] cubeNodeArray = new Pose2d[] {BLUE2, BLUE5, BLUE8, RED2, RED5, RED8};
    private static final Pose2d[] coneNodeArray = new Pose2d[] {BLUE1, BLUE3, BLUE4, BLUE6, BLUE7, BLUE9, RED1, RED3, RED4, RED6, RED7, RED9};
    
    private static final Pose2d[] nodeArrayBlue = new Pose2d[] {BLUE1, BLUE2, BLUE3, BLUE4, BLUE5, BLUE6, BLUE7, BLUE8, BLUE9, BLUE_RAMP, BLUE_SHELF_LEFT, BLUE_SHELF_RIGHT};
    private static final Pose2d[] cubeNodeArrayBlue = new Pose2d[] {BLUE2, BLUE5, BLUE8, BLUE_RAMP, BLUE_SHELF_LEFT, BLUE_SHELF_RIGHT};
    private static final Pose2d[] coneNodeArrayBlue = new Pose2d[] {BLUE1, BLUE3, BLUE4, BLUE6, BLUE7, BLUE9, BLUE_RAMP, BLUE_SHELF_LEFT, BLUE_SHELF_RIGHT};
    
    private static final Pose2d[] nodeArrayRed = new Pose2d[] {RED1, RED2, RED3, RED4, RED5, RED6, RED7, RED8, RED9, RED_RAMP, RED_SHELF_LEFT, RED_SHELF_RIGHT};
    private static final Pose2d[] cubeNodeArrayRed = new Pose2d[] {RED2, RED5, RED8, RED_RAMP, RED_SHELF_LEFT, RED_SHELF_RIGHT};
    private static final Pose2d[] coneNodeArrayRed = new Pose2d[] {RED1, RED3, RED4, RED6, RED7, RED9, RED_RAMP, RED_SHELF_LEFT, RED_SHELF_RIGHT};
    
    public static final List<Pose2d> ALL_NODES = Arrays.asList(nodeArray);
    public static final List<Pose2d> ALL_NODES_BLUE = Arrays.asList(nodeArrayBlue);
    public static final List<Pose2d> ALL_NODES_RED = Arrays.asList(nodeArrayRed);
    public static final List<Pose2d> ALL_NODES_CUBE_RED = Arrays.asList(cubeNodeArrayRed);
    public static final List<Pose2d> ALL_NODES_CONE_RED = Arrays.asList(coneNodeArrayRed);
    public static final List<Pose2d> ALL_NODES_CUBE_BLUE = Arrays.asList(cubeNodeArrayBlue);
    public static final List<Pose2d> ALL_NODES_CONE_BLUE = Arrays.asList(coneNodeArrayBlue);

  }
}
