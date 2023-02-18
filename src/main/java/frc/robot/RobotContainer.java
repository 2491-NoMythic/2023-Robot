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
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.Drive;
import frc.robot.Commands.DriveRotateToAngleCommand;
import frc.robot.Commands.EndEffectorCommand;
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
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
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
  
  private RobotArmSubsystem arm;
  private RobotArmControl ControlArm;
  
  private EndEffectorCommand endEffectorCommand;
  private SkiPlowPneumatic skiplowcommand; 
  private SkiPlow skiPlow;

  private EndEffector effector;
  
  private SubsystemLights lightsSubsystem;

  public static boolean ArmExists = Preferences.getBoolean("Arm", false);
  public static boolean EndEffectorExists = Preferences.getBoolean("EndEffector", false);
  public static boolean SkiPlowExists = Preferences.getBoolean("SkiPlow", false);
  public static boolean LimelightExists = Preferences.getBoolean("Limelight", false);
  public static boolean LimelightMotorExists = Preferences.getBoolean("LimelightMotor", false);
  public static boolean DrivetrainExists = Preferences.getBoolean("Drivetrain", false);
  public static boolean LightsExists = Preferences.getBoolean("Lights", false);

      
  
  public RobotContainer() {
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
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

    if (ArmExists){
      ArmInst();
    }
    if (EndEffectorExists){
      EndEffectorInst();
    }
    if (SkiPlowExists){
      SkiPlowInst();
    }
    if (LimelightExists){
      LimelightInst();
    }
    if (LimelightMotorExists){
      LimelightMotorInst();
    }
    if (DrivetrainExists){
      DrivetrainInst();
    }
    if (LightsExists){
      LightsInst();
    }
    
    
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> Robot X movement, backward/forward
    // Left stick X axis -> Robot Y movement, right/left
    // Right stick Z axis -> rotation, clockwise, counterclockwise
    // Need to invert the joystick axis
    configureBindings();
    configDashboard();
  } 
  
  private void LightsInst() {
    lightsSubsystem = new SubsystemLights(60);
  }

  private void DrivetrainInst(){
    drivetrain = new DrivetrainSubsystem();
    defaultDriveCommand = new Drive(
      drivetrain,
      () -> driveController.getL1Button(),
      () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driveController.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
    drivetrain.setDefaultCommand(defaultDriveCommand);
    Command defaultllmotorCommand = new RunViaLimelightCommand(llmotor);
    configureBindings();
    configDashboard();
    llmotor.setDefaultCommand(defaultllmotorCommand);
    arm.setDefaultCommand(ControlArm);
    effector.setDefaultCommand(endEffectorCommand);
    skiPlow.setDefaultCommand(skiplowcommand);
  } 
  private void configDashboard() {

    }
  private void ArmInst(){
    arm = new RobotArmSubsystem();
    ControlArm = new RobotArmControl(arm);
    arm.setDefaultCommand(ControlArm);
  }
  private void EndEffectorInst(){
    effector = new EndEffector();
    endEffectorCommand = new EndEffectorCommand(effector, opController);
    effector.setDefaultCommand(endEffectorCommand);
  }
  private void SkiPlowInst(){
    skiPlow = new SkiPlow();
    skiPlow.setDefaultCommand(skiplowcommand);  
    skiplowcommand = new SkiPlowPneumatic(skiPlow, opController);
  }
  private void LimelightInst(){
    limelight = Limelight.getInstance();
  }
  private void LimelightMotorInst(){
    llmotor = new LimelightmotorSubsystem();
    Command defaultllmotorCommand = new RunViaLimelightCommand(llmotor);
    llmotor.setDefaultCommand(defaultllmotorCommand);
  }
  
  
  
  /**Takes both axis of a joystick, returns an angle from -180 to 180 degrees, or {@link Constants.PS4Driver.NO_INPUT} (double = 404.0) if the joystick is at rest position*/
  private double getJoystickDegrees(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_LARGE);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_LARGE);
    if (xAxis + yAxis != 0) {
      return Math.toDegrees(Math.atan2(xAxis, yAxis));
    }
    return NO_INPUT;
  }
  /**Takes both axis of a joystick, returns a double from 0-1 */
  private double getJoystickMagnitude(int horizontalAxis, int verticalAxis) {
    double xAxis = MathUtil.applyDeadband(-driveController.getRawAxis(horizontalAxis), DEADBAND_NORMAL);
    double yAxis = MathUtil.applyDeadband(-driveController.getRawAxis(verticalAxis), DEADBAND_NORMAL);
    return Math.min(1.0, (Math.sqrt(Math.pow(xAxis, 2) + Math.pow(yAxis, 2)))); // make sure the number is not greater than 1
  }
  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4DriverController
   * PS4Driver} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // Schedule `exampleMethodCommand` when the Xbox driveController's B button is pressed,
    // cancelling on release.
    
      if (DrivetrainExists){
        new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(drivetrain::zeroGyroscope, drivetrain));
        //new Trigger(driveController::getTriangleButton).onTrue(Commands.runOnce(() -> this.moveToPose(DriveConstants.DRIVE_ODOMETRY_ORIGIN)));
        new Trigger(driveController::getR1Button).whileTrue(new DriveRotateToAngleCommand(drivetrain, 
      () -> modifyAxis(-driveController.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
      () -> modifyAxis(-driveController.getRawAxis(X_AXIS), DEADBAND_NORMAL),
      () -> getJoystickDegrees(Z_AXIS, Z_ROTATE),
      () -> getJoystickMagnitude(Z_AXIS, Z_ROTATE)));
      }
      if (LightsExists){
        new Trigger(driveController::getTriangleButton).onTrue(Commands.runOnce(()->  {lightsSubsystem.lightsOut(); lightsSubsystem.setLights(29, 59, 100, 100, 0);}, lightsSubsystem));
      new Trigger(driveController::getSquareButton).onTrue(Commands.runOnce(()->  {lightsSubsystem.lightsOut(); lightsSubsystem.setLights(0, 39, 100, 0, 100);}, lightsSubsystem));
      new Trigger(driveController::getPSButton).onTrue(Commands.runOnce(()->  {lightsSubsystem.lightsOut(); lightsSubsystem.setLights(0, 59, 0, 0, 0);}, lightsSubsystem));
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
   private static double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }
  public void robotInit() {
    PathPlannerServer.startServer(5811);
    if(DrivetrainExists){
      drivetrain.zeroGyroscope();
    }
  }
  public void teleopInit() {
    if(DrivetrainExists){
      drivetrain.pointWheelsForward();
    }
  }
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Match Timer", Timer.getMatchTime());
  }
  public void moveToPose(Pose2d targetPose) {
 		Pose2d currentPose = drivetrain.getPose();
		PathPlannerTrajectory newTraj = PathPlanner.generatePath(
				new PathConstraints(3, 1.5),
        new PathPoint(currentPose.getTranslation(), currentPose.relativeTo(targetPose).getTranslation().getAngle(), currentPose.getRotation()),
				new PathPoint(targetPose.getTranslation(), currentPose.relativeTo(targetPose).getTranslation().getAngle(), targetPose.getRotation()));
		Command followTraj = drivetrain.followPPTrajectory(newTraj, false);
    drivetrain.m_field.getObject("traj").setTrajectory(newTraj);
		followTraj.schedule();
  }
}
