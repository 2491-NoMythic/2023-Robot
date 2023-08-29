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

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmMoveAxis;
import frc.robot.Commands.Autos;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveBalanceCommand;
import frc.robot.Commands.DriveOffsetCenterCommand;
import frc.robot.Commands.DriveRotateToAngleCommand;
import frc.robot.Commands.DriveToBalance;
import frc.robot.Commands.EndEffectorCommand;
import frc.robot.Commands.EndEffectorPassiveCommand;
import frc.robot.Commands.EndEffectorRun;
import frc.robot.Commands.LightsByModeCommand;
import frc.robot.Commands.PurpleLights;
import frc.robot.Commands.ScoreNearestNodeHigh;
import frc.robot.Commands.ScoreNearestNodeLow;
import frc.robot.Commands.ScoreNearestNodeMid;
import frc.robot.Commands.arm.RampCone;
import frc.robot.Commands.arm.RampCube;
import frc.robot.Commands.arm.DropLow;
import frc.robot.Commands.arm.HighCone;
import frc.robot.Commands.arm.HighCube;
import frc.robot.Commands.arm.IntakeCone;
import frc.robot.Commands.arm.IntakeCube;
import frc.robot.Commands.arm.MidCone;
import frc.robot.Commands.arm.MidCube;
import frc.robot.Commands.arm.Reset;
import frc.robot.Commands.arm.ResetFast;
import frc.robot.Commands.arm.ShelfCone;
import frc.robot.Commands.arm.ShelfCube;
import frc.robot.settings.Constants;

import frc.robot.settings.IntakeState;
import frc.robot.settings.Constants.Arm;
import frc.robot.settings.IntakeState.IntakeMode;

import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SkiPlow;
import frc.robot.subsystems.SubsystemLights;

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

  private DrivetrainSubsystem drivetrain;
  private Drive defaultDriveCommand;
  private SendableChooser<Command> autoChooser;
  private final PS4Controller driveController;
  private final PS4Controller opController;

  private Autos autos;

  private ArmSubsystem arm;
  private ArmMoveAxis armDefault;
  // private RobotArmControl ControlArm;


  private EndEffectorPassiveCommand endEffectorPassiveCommand;
  private SkiPlow skiPlow;

  private EndEffector effector;

  private SubsystemLights lightsSubsystem;

  public static HashMap<String, Command> eventMap;
  public static Map<Object, Command> moveToIntakePose;
  public static boolean ArmExists = Preferences.getBoolean("Arm", false);
  public static boolean EndEffectorExists = Preferences.getBoolean("EndEffector", false);
  public static boolean SkiPlowExists = Preferences.getBoolean("SkiPlow", false);
  public static boolean LimelightExists = Preferences.getBoolean("Limelight", false);
  public static boolean DrivetrainExists = Preferences.getBoolean("Drivetrain", false);
  public static boolean LightsExists = Preferences.getBoolean("Lights", false);

  public IntakeState intakeState;

  public RobotContainer() {
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    Preferences.initBoolean("Arm", false);
    Preferences.initBoolean("Drivetrain", false);
    Preferences.initBoolean("EndEffector", false);
    Preferences.initBoolean("SkiPlow", false);
    Preferences.initBoolean("Limelight", false);
    Preferences.initBoolean("Lights", false);
    driveController = new PS4Controller(0);
    opController = new PS4Controller(1);
    autoChooser = new SendableChooser<>();
    eventMap = new HashMap<>();
    intakeState = IntakeState.getInstance();

    // SmartDashboard.putNumber("endeffectorBigSpeed", 0.5);
    // SmartDashboard.putNumber("endeffectorSmallSpeed", 0.75);
    // SmartDashboard.putNumber("skiplowRollerSpeed", 0.5);

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
    if (DrivetrainExists) {
      DrivetrainInst();
    }
    if (LightsExists) {
      LightsInst();
    }
    if(ArmExists) {
      moveToIntakePose = Map.ofEntries(
        Map.entry(IntakeMode.CONE_GROUND, new IntakeCone(arm)),
        Map.entry(IntakeMode.CONE_SHELF, new ShelfCone(arm)),
        Map.entry(IntakeMode.CONE_RAMP, new RampCone(arm)),
        Map.entry(IntakeMode.CUBE_GROUND, new IntakeCube(arm)),
        Map.entry(IntakeMode.CUBE_SHELF, new ShelfCube(arm)),
        Map.entry(IntakeMode.CUBE_RAMP, new RampCube(arm))
      );
      // SmartDashboard.putData("GoTo Intake Cone", new IntakeCone(arm, skiPlow));
      // SmartDashboard.putData("GoTo Intake Cube", new IntakeCube(arm, skiPlow));
      // SmartDashboard.putData("GoTo Reset", new Reset(arm, skiPlow));
      // SmartDashboard.putData("GoTo High Cone", new HighCone(arm, skiPlow));
    }
    autoInit();
    configureBindings();
    configDashboard();
  }

  private void LightsInst() {
    lightsSubsystem = new SubsystemLights(52);
    PurpleLights defaultLights = new PurpleLights(lightsSubsystem);
    LightsByModeCommand lightsByMode = new LightsByModeCommand(lightsSubsystem, ArmExists);
    lightsSubsystem.setDefaultCommand(lightsByMode);
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
    SmartDashboard.putData(new DriveToBalance(drivetrain, true));
    SmartDashboard.putData(drivetrain);
    SmartDashboard.putNumber("Precision Multiplier", 0.2);
    SmartDashboard.putBoolean("Safe Mode", true);
  }

  private void ArmInst(){
    arm = new ArmSubsystem();
    armDefault = new ArmMoveAxis(arm, ()->opController.getRawAxis(Y_AXIS), ()->opController.getRawAxis(Z_ROTATE));
    arm.setDefaultCommand(armDefault);
  }
  private void EndEffectorInst(){
    effector = new EndEffector(Arm.END_EFFECTOR_BIG_POWER, Arm.END_EFFECTOR_SMALL_POWER);
    endEffectorPassiveCommand = new EndEffectorPassiveCommand(effector);
    effector.setDefaultCommand(endEffectorPassiveCommand);
  }
  private void SkiPlowInst(){
    // skiPlow = new SkiPlow(0.5);
    // skiPlow = new SkiPlow(SmartDashboard.getNumber("skiplowRollerSpeed", 0.5));
  }

  private void LimelightInst() {
    limelight = Limelight.getInstance();
    SmartDashboard.putBoolean("UseLimelight", SmartDashboard.getBoolean("UseLimelight", true));
    SmartDashboard.putBoolean("ForceTrustLimelight", SmartDashboard.getBoolean("ForceTrustLimelight", false));
  }

  private void autoInit() {
    autos = Autos.getInstance();
    if (DrivetrainExists) {
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      eventMap.put("marker2", new PrintCommand("Passed marker 2"));
      eventMap.put("stop", new InstantCommand(drivetrain::stop, drivetrain));
      eventMap.put("Wait1", new WaitCommand(1));
      eventMap.put("Waithalf", new WaitCommand(0.5));
      // if (SkiPlowExists) {
        // eventMap.put("IntakeDown", new SequentialCommandGroup(new InstantCommand(skiPlow::pistonDown, skiPlow), new WaitCommand(0.75)));
        // eventMap.put("IntakeUp", new SequentialCommandGroup(new InstantCommand(skiPlow::pistonUp, skiPlow), new WaitCommand(0.5)));
        // eventMap.put("IntakeRollerIn", new SequentialCommandGroup(new InstantCommand(skiPlow::rollerCube, skiPlow)));
        // eventMap.put("IntakeRollerOut", new SequentialCommandGroup(new InstantCommand(skiPlow::rollerCone, skiPlow)));
        // eventMap.put("IntakeOff", new SequentialCommandGroup(new InstantCommand(skiPlow::rollerOff, skiPlow)));
      // }
      if (EndEffectorExists) {
        eventMap.put("Outtake", new ParallelDeadlineGroup(new WaitCommand(.25), new EndEffectorCommand(effector, ()->false)));
        eventMap.put("Intake", new EndEffectorRun(effector, ()->true));
        eventMap.put("EndEffectorStop", new InstantCommand(effector::rollerOff, effector));
      }
      if (LightsExists) {
        // eventMap.put("LightsOff", new SequentialCommandGroup(new InstantCommand(lightsSubsystem::lightsOut, lightsSubsystem)));
      }
      if (LimelightExists) {
        eventMap.put("TrustLimelightTrue", Commands.runOnce(()-> DrivetrainSubsystem.forceTrustLimelight(true)));
        eventMap.put("TrustLimelightFalse", Commands.runOnce(()-> DrivetrainSubsystem.forceTrustLimelight(false)));
      }
      if (ArmExists) {
        eventMap.put("CubeShelf", new ShelfCube(arm));
        eventMap.put("ConeShelf", new ShelfCone(arm));
        eventMap.put("CubeFloor", new IntakeCube(arm));
        eventMap.put("ConeFloor", new IntakeCone(arm));

        eventMap.put("ScoreHigh", Commands.either(new HighCone(arm), new HighCube(arm), intakeState::isConeMode).andThen(Commands.waitSeconds(0.25)));
        eventMap.put("ScoreMid", Commands.either(new MidCone(arm), new MidCube(arm), intakeState::isConeMode).andThen(Commands.waitSeconds(0.25)));
        eventMap.put("ScoreLow", new DropLow(arm));
        
        eventMap.put("ResetArmPose", new Reset(arm));
        eventMap.put("ResetArmPoseFast", new ResetFast(arm));
      }
      eventMap.put("ModeCubeGround", Commands.runOnce(()->IntakeState.setIntakeMode(IntakeMode.CUBE_GROUND)).alongWith(Commands.run(effector::rollerInCube).withTimeout(0.25)));
      eventMap.put("ModeCubeShelf", Commands.runOnce(()->IntakeState.setIntakeMode(IntakeMode.CUBE_SHELF)).alongWith(Commands.run(effector::rollerInCube).withTimeout(0.25)));
      eventMap.put("ModeConeGround", Commands.runOnce(()->IntakeState.setIntakeMode(IntakeMode.CONE_GROUND)).alongWith(Commands.run(effector::rollerInCone).withTimeout(0.25)));
      eventMap.put("ModeConeRamp", Commands.runOnce(()->IntakeState.setIntakeMode(IntakeMode.CONE_RAMP)));
      eventMap.put("ModeConeShelf", Commands.runOnce(()->IntakeState.setIntakeMode(IntakeMode.CONE_SHELF)).alongWith(Commands.run(effector::rollerInCone).withTimeout(0.25)));
      autos.autoInit(autoChooser, eventMap, drivetrain);
      SmartDashboard.putData(autoChooser);
    }
  }

  private void configDashboard() {
    SmartDashboard.putBoolean("ConeMode", false);
    SmartDashboard.putBoolean("CubeMode", false);
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
      new Trigger(driveController::getSquareButton).whileTrue(new DriveBalanceCommand(drivetrain));
      new Trigger(driveController::getTriangleButton).onTrue(Commands.runOnce(()->autos.moveToPose(drivetrain.getNearestNode()).schedule()));
      new Trigger(driveController::getCrossButton).onTrue(Commands.runOnce(drivetrain::pointWheelsInward, drivetrain));
      // new Trigger(driveController::getR1Button).whileTrue(new DriveRotateToAngleCommand(drivetrain,
      //     () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      //     () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      //     () -> getJoystickDegrees(Z_AXIS, Z_ROTATE),
      //     () -> getJoystickMagnitude(Z_AXIS, Z_ROTATE)));
      // new Trigger(driveController::getR2Button).whileTrue(new DriveOffsetCenterCommand(
      //   drivetrain,
      //   () -> driveController.getL1Button(),
      //   () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      //   () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      //   () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL), 
      //   new Translation2d(1,0)));
    }
    if (ArmExists) {
      
      new Trigger(opController::getR1Button)
          .onTrue(Commands.select(moveToIntakePose, intakeState::getIntakeMode));
      new Trigger(opController::getR2Button)
          .whileTrue(new EndEffectorCommand(effector, ()->true));
      new Trigger(opController::getL2Button)
          .whileTrue(new EndEffectorCommand(effector, ()->false));
      new Trigger(opController::getTouchpadPressed)
          .onTrue(new Reset(arm));
          
      new Trigger(()-> opController.getPOV() == 0)
          .onTrue(Commands.either(new HighCone(arm), new HighCube(arm), intakeState::isConeMode));//score high
      new Trigger(()-> opController.getPOV() == 90).or(()-> opController.getPOV() == 270)
          .onTrue(Commands.either(new MidCone(arm), new MidCube(arm), intakeState::isConeMode));//score mid
      new Trigger(()-> opController.getPOV() == 180)
          .onTrue(new DropLow(arm));//score floor
    }

    if (SkiPlowExists) {
      // BooleanSupplier tmp = opController::getR1Button;
      // BooleanSupplier tmp2 = () -> opController.getR1Button();
    }
    if (LimelightExists) {
      if (DrivetrainExists) {
        new Trigger(()->SmartDashboard.getBoolean("UseLimelight", true))
          .onTrue(Commands.runOnce(()-> DrivetrainSubsystem.useLimelight(true)))
          .onFalse(Commands.runOnce(()-> DrivetrainSubsystem.useLimelight(false)));
        new Trigger(driveController::getTouchpad)
          .onTrue(Commands.runOnce(()-> DrivetrainSubsystem.forceTrustLimelight(true)))
          .onFalse(Commands.runOnce(()-> DrivetrainSubsystem.forceTrustLimelight(false)));
      }
    }
    new Trigger(opController::getOptionsButton)
        .onTrue(Commands.runOnce(() -> IntakeState.setIntakeMode(IntakeMode.CUBE_GROUND)))
        .onTrue(Commands.runOnce(() -> {
          SmartDashboard.putBoolean("ConeMode", false);
          SmartDashboard.putBoolean("CubeMode", true);
        }));
    new Trigger(opController::getSquareButton)
        .onTrue(Commands.runOnce(() -> IntakeState.setIntakeMode(IntakeMode.CUBE_RAMP)))
        .onTrue(Commands.runOnce(() -> {
          SmartDashboard.putBoolean("ConeMode", false);
          SmartDashboard.putBoolean("CubeMode", true);
        }));
    new Trigger(opController::getCrossButton)
        .onTrue(Commands.runOnce(() -> IntakeState.setIntakeMode(IntakeMode.CUBE_SHELF)))
        .onTrue(Commands.runOnce(() -> {
          SmartDashboard.putBoolean("ConeMode", false);
          SmartDashboard.putBoolean("CubeMode", true);
        }));
    
    new Trigger(opController::getTriangleButton)
        .onTrue(Commands.runOnce(() -> IntakeState.setIntakeMode(IntakeMode.CONE_RAMP)))
        .onTrue(Commands.runOnce(() -> {
          SmartDashboard.putBoolean("ConeMode", true);
          SmartDashboard.putBoolean("CubeMode", false);
        }));
    new Trigger(opController::getCircleButton)
        .onTrue(Commands.runOnce(() -> IntakeState.setIntakeMode(IntakeMode.CONE_SHELF)))
        .onTrue(Commands.runOnce(() -> {
          SmartDashboard.putBoolean("ConeMode", true);
          SmartDashboard.putBoolean("CubeMode", false);
        }));
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
    if (driveController.getL2Button()|| driveController.getL1Button() || opController.getL1Button() || SmartDashboard.getBoolean("Safe Mode", true)) {
      value *= SmartDashboard.getNumber("Precision Multiplier", 0.2);
    }
    else if (ArmExists){
      if (Math.abs(arm.getArmPose().getX()) >= 0.5) {
        value *= 0.2;
      }
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
    // SmartDashboard.putString("NearestNode", drivetrain.getPose().nearest(ALL_NODES).toString());
    SmartDashboard.putNumber("Match Timer", Timer.getMatchTime());
  }
}
