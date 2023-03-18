// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.PS4Driver.DEADBAND_LARGE;
import static frc.robot.settings.Constants.PS4Driver.DEADBAND_NORMAL;
import static frc.robot.settings.Constants.PS4Driver.NO_INPUT;
import static frc.robot.settings.Constants.PS4Driver.X_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Y_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_AXIS;
import static frc.robot.settings.Constants.PS4Driver.Z_ROTATE;

import java.sql.DriverAction;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Autos;

import frc.robot.Commands.DriveBalanceCommand;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveOffsetCenterCommand;
import frc.robot.Commands.DriveRotateToAngleCommand;
import frc.robot.Commands.EndEffectorCommand;
import frc.robot.Commands.PurpleLights;
import frc.robot.Commands.RobotArmControl;
import frc.robot.Commands.RunViaLimelightCommand;
import frc.robot.Commands.SkiPlowPneumatic;
import frc.robot.settings.Constants;
import frc.robot.settings.Constants.DriveConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightmotorSubsystem;
import frc.robot.subsystems.RobotArmSubsystem;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.subsystems.SkiPlow;
import frc.robot.subsystems.SubsystemLights;
import frc.robot.subsystems.SkiPlow;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private Limelight limelight;
  private LimelightmotorSubsystem llmotor;

  private DrivetrainSubsystem drivetrain;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final PS4Controller driveController;
  private final PS4Controller opController;

  private Autos autos;

  private RobotArmSubsystem arm;
  private RobotArmControl ControlArm;

  private EndEffectorCommand endEffectorCommand;
  private SkiPlowPneumatic skiplowcommand;
  private SkiPlow skiPlow;

  private EndEffector effector;

  private SubsystemLights lightsSubsystem;

  public static HashMap<String, Command> eventMap;
  public static boolean ArmExists = Preferences.getBoolean("Arm", false);
  public static boolean EndEffectorExists = Preferences.getBoolean("EndEffector", false);
  public static boolean SkiPlowExists = Preferences.getBoolean("SkiPlow", false);
  public static boolean LimelightExists = Preferences.getBoolean("Limelight", false);
  public static boolean LimelightMotorExists = Preferences.getBoolean("LimelightMotor", false);
  public static boolean DrivetrainExists = Preferences.getBoolean("Drivetrain", false);
  public static boolean LightsExists = Preferences.getBoolean("Lights", false);

  public RobotContainer() {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    Preferences.initBoolean("Arm", false);
    Preferences.initBoolean("Drivetrain", false);
    Preferences.initBoolean("EndEffector", false);
    Preferences.initBoolean("SkiPlow", false);
    Preferences.initBoolean("Limelight", false);
    Preferences.initBoolean("LimelightMotor", false);
    Preferences.initBoolean("Lights", false);
    driveController = new PS4Controller(0);
    opController = new PS4Controller(1);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();

    SmartDashboard.putNumber("endeffectorSpeed", 0.5);
    SmartDashboard.putNumber("skiplowRollerSpeed", 0.5);

    if (ArmExists) {
      ArmInst();
    }
    if (EndEffectorExists) {
      EndEffectorInst();
    }
    if (SkiPlowExists) {
      SkiPlowInst();
    }
    if (LimelightExists) {
      LimelightInst();
    }
    if (LimelightMotorExists) {
      LimelightMotorInst();
    }
    if (DrivetrainExists) {
      DrivetrainInst();
    }
    if (LightsExists) {
      LightsInst();
    }
    autoInit();
    configureBindings();
    configDashboard();
  }

  private void LightsInst() {
    lightsSubsystem = new SubsystemLights(52);
    PurpleLights defaultLights = new PurpleLights(lightsSubsystem);
    lightsSubsystem.setDefaultCommand(defaultLights);
  }

  private void DrivetrainInst() {
    drivetrain = new DrivetrainSubsystem();
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> Robot X movement, backward/forward
    // Left stick X axis -> Robot Y movement, right/left
    // Right stick Z axis -> rotation, clockwise, counterclockwise
    // Need to invert the joystick axis
    defaultDriveCommand = new Drive(
        drivetrain,
        () -> driveController.getL1Button(),
        () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
        () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
        () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
    drivetrain.setDefaultCommand(defaultDriveCommand);
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putNumber("Robot origin x", DriveConstants.DRIVE_ODOMETRY_ORIGIN.getX());
    SmartDashboard.putNumber("Robot origin y", DriveConstants.DRIVE_ODOMETRY_ORIGIN.getY());
    SmartDashboard.putNumber("Robot origin rot", DriveConstants.DRIVE_ODOMETRY_ORIGIN.getRotation().getDegrees());
    SmartDashboard.putNumber("Precision Multiplier", 0.5);
  }

  private void ArmInst() {
    arm = new RobotArmSubsystem();
    ControlArm = new RobotArmControl(arm, 
    () -> opController.getRightY(), 
    () -> opController.getLeftY());
    arm.setDefaultCommand(ControlArm);
  }
  private void EndEffectorInst(){
    effector = new EndEffector(SmartDashboard.getNumber("endeffectorSpeed", 0.5));
    endEffectorCommand = new EndEffectorCommand(effector, 
    () -> opController.getRightY(), 
    SmartDashboard.getNumber("endeffectorSpeed", 0.5));

  }
  private void SkiPlowInst(){
    skiPlow = new SkiPlow(SmartDashboard.getNumber("skiplowRollerSpeed", 0.5));
    skiplowcommand = new SkiPlowPneumatic(skiPlow, 
    opController::getL2Button, 
    opController::getCrossButton, 
    opController::getTriangleButton,
    opController::getSquareButton,
    SmartDashboard.getNumber("skiplowRollerSpeed", 0.5)
    );
    skiPlow.setDefaultCommand(skiplowcommand);  
  }

  private void LimelightInst() {
    limelight = Limelight.getInstance();
  }

  private void LimelightMotorInst() {
    llmotor = new LimelightmotorSubsystem();
    Command defaultllmotorCommand = new RunViaLimelightCommand(llmotor);
    llmotor.setDefaultCommand(defaultllmotorCommand);
  }

  private void autoInit() {
    autos = Autos.getInstance();
    if (DrivetrainExists) {
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("marker2", new PrintCommand("Passed marker 2"));
      eventMap.put("stop", new InstantCommand(drivetrain::stop, drivetrain));
      if (SkiPlowExists) {
        eventMap.put("IntakeDown", new SequentialCommandGroup(new InstantCommand(skiPlow::pistonDown, skiPlow), new WaitCommand(0.75)));
        eventMap.put("IntakeUp", new SequentialCommandGroup(new InstantCommand(skiPlow::pistonUp, skiPlow), new WaitCommand(0.5)));
        eventMap.put("IntakeRollerIn", new SequentialCommandGroup(new InstantCommand(skiPlow::rollerCube, skiPlow)));
        eventMap.put("IntakeOut", new SequentialCommandGroup(new InstantCommand(skiPlow::rollerCone, skiPlow)));
        // eventMap.put("skiPlowLock", TODO add command);
      }
      if (EndEffectorExists) {
        eventMap.put("EndEffectorIn", new SequentialCommandGroup(new InstantCommand(effector::rollerIn, effector)));
        eventMap.put("EndEffectorOut", new SequentialCommandGroup(new InstantCommand(effector::rollerOut, effector)));
      }
      if (LightsExists) {
        // eventMap.put("TODO add command", TODO add command);
        // eventMap.put("TODO add command", TODO add command);
      }
      if (ArmExists) {
        // eventMap.put("armPoint1", TODO add command);
        // eventMap.put("armPoint2", TODO add command);
      }
      autos.autoInit(autoChooser, eventMap, drivetrain);
      SmartDashboard.putData(autoChooser);
    }
  }

  private void configDashboard() {
  }

  /**
   * Takes both axis of a joystick, returns an angle from -180 to 180 degrees, or
   * {@link Constants.PS4Driver.NO_INPUT} (double = 404.0) if the joystick is at
   * rest position
   */
  private double getJoystickDegrees(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_LARGE);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_LARGE);
    if (xAxis + yAxis != 0) {
      return Math.toDegrees(Math.atan2(xAxis, yAxis));
    }
    return NO_INPUT;
  }

  /** Takes both axis of a joystick, returns a double from 0-1 */
  private double getJoystickMagnitude(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_NORMAL);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_NORMAL);
    return Math.min(1.0, (Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2)))); // make sure the number is not greater
                                                                                // than 1
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4DriverController
   * PS4Driver} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    if (DrivetrainExists) {
      new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
      // new Trigger(driveController::getTriangleButton).onTrue(Commands.runOnce(() ->
      // autos.moveToPose(DriveConstants.DRIVE_ODOMETRY_ORIGIN)));
      new Trigger(driveController::getCrossButton).onTrue(Commands.runOnce(drivetrain::pointWheelsInward, drivetrain));
      new Trigger(driveController::getR1Button).whileTrue(new DriveRotateToAngleCommand(drivetrain,
          () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
          () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
          () -> getJoystickDegrees(Z_AXIS, Z_ROTATE),
          () -> getJoystickMagnitude(Z_AXIS, Z_ROTATE)));
      new Trigger(driveController::getR2Button).whileTrue(new DriveOffsetCenterCommand(
          drivetrain,
          () -> driveController.getL1Button(),
          () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
          () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
          () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL),
          new Translation2d(1, 0)));
      new Trigger(driveController::getSquareButton).whileTrue(new DriveBalanceCommand(
          drivetrain));

    }
    if (LightsExists) {
      new Trigger(opController::getTriangleButton).whileTrue(Commands.run(() -> {
        lightsSubsystem.lightsOut();
        lightsSubsystem.setLights(0, 51, 100, 64, 0);
      }, lightsSubsystem));
      new Trigger(opController::getSquareButton).whileTrue(Commands.run(() -> {
        lightsSubsystem.lightsOut();
        lightsSubsystem.setLights(0, 51, 0, 0, 100);
      }, lightsSubsystem));
    }
    if (ArmExists) {
      new Trigger(opController::getL1Button).onTrue(Commands.runOnce(() -> {
        arm.setShoulderPower(-0.1);
      }, arm)).onFalse(Commands.runOnce(() -> {
        arm.setShoulderPower(0);
      }, arm));
      new Trigger(opController::getL2Button).onTrue(Commands.runOnce(() -> {
        arm.setShoulderPower(0.1);
      }, arm)).onFalse(Commands.runOnce(() -> {
        arm.setShoulderPower(0);
      }, arm));
      new Trigger(opController::getShareButton).onTrue(Commands.runOnce(() -> {
        arm.setElbowPower(-0.1);
      }, arm)).onFalse(Commands.runOnce(() -> {
        arm.setElbowPower(0);
      }, arm));
      new Trigger(opController::getOptionsButton).onTrue(Commands.runOnce(() -> {
        arm.setElbowPower(0.1);
      }, arm)).onFalse(Commands.runOnce(() -> {
        arm.setElbowPower(0);
      }, arm));
    }
    if (SkiPlowExists) {
      BooleanSupplier tmp = opController::getR1Button;
      BooleanSupplier tmp2 = () -> opController.getR1Button();
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    if (driveController.getL2Button()) {
      value *= SmartDashboard.getNumber("Precision Multiplier", 0.3);
    }
    return value;
  }

  public void robotInit() {
    if (DrivetrainExists) {
      drivetrain.zeroGyroscope();
    }
  }

  public void teleopInit() {
    if (DrivetrainExists) {
      drivetrain.pointWheelsForward();
    }
  }

  public void teleopPeriodic() {
    SmartDashboard.putNumber("Match Timer", Timer.getMatchTime());
  }
}
