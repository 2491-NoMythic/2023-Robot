package frc.robot.settings;

import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_DRIVE_INVERTED;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_DRIVE_REDUCTION;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_STEER_INVERTED;
import static frc.robot.settings.Constants.DriveConstants.DRIVETRAIN_STEER_REDUCTION;
import static frc.robot.settings.Constants.DriveConstants.DRIVE_MOTOR_RAMP;
import static frc.robot.settings.Constants.DriveConstants.k_DRIVE_D;
import static frc.robot.settings.Constants.DriveConstants.k_DRIVE_FF_S;
import static frc.robot.settings.Constants.DriveConstants.k_DRIVE_FF_V;
import static frc.robot.settings.Constants.DriveConstants.k_DRIVE_I;
import static frc.robot.settings.Constants.DriveConstants.k_DRIVE_P;
import static frc.robot.settings.Constants.DriveConstants.k_STEER_D;
import static frc.robot.settings.Constants.DriveConstants.k_STEER_FF_S;
import static frc.robot.settings.Constants.DriveConstants.k_STEER_FF_V;
import static frc.robot.settings.Constants.DriveConstants.k_STEER_I;
import static frc.robot.settings.Constants.DriveConstants.k_STEER_P;

import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.Pigeon2Configuration;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

public class CTREConfigs {
    public TalonFXConfiguration driveMotorConfig;
    public TalonFXConfiguration steerMotorConfig;
    public CANcoderConfiguration steerEncoderConfig;
    public Pigeon2Configuration pigeon2Config;

    public CTREConfigs() {
        driveMotorConfig = new TalonFXConfiguration();
        steerMotorConfig = new TalonFXConfiguration();
        steerEncoderConfig = new CANcoderConfiguration();
        pigeon2Config = new Pigeon2Configuration();

        // Steer motor.
        steerMotorConfig.Feedback.RotorToSensorRatio = 1/DRIVETRAIN_STEER_REDUCTION;
        steerMotorConfig.MotorOutput.Inverted = DRIVETRAIN_STEER_INVERTED;
        // steerMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.05;
        steerMotorConfig.Slot0.kP = k_STEER_P;
        steerMotorConfig.Slot0.kI = k_STEER_I;
        steerMotorConfig.Slot0.kD = k_STEER_D;
        steerMotorConfig.Slot0.kS = k_STEER_FF_S;
        steerMotorConfig.Slot0.kV = k_STEER_FF_V;
        steerMotorConfig.Voltage.PeakForwardVoltage = 12;
        steerMotorConfig.Voltage.PeakReverseVoltage = -12;
        steerMotorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;

        steerMotorConfig.ClosedLoopGeneral.ContinuousWrap = true;
        // Drive motor.
        driveMotorConfig.Feedback.SensorToMechanismRatio = 1/DRIVETRAIN_DRIVE_REDUCTION;
        driveMotorConfig.MotorOutput.Inverted = DRIVETRAIN_DRIVE_INVERTED;
        driveMotorConfig.MotorOutput.DutyCycleNeutralDeadband = 0.0;
        driveMotorConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = DRIVE_MOTOR_RAMP;
        driveMotorConfig.Slot0.kP = k_DRIVE_P;
        driveMotorConfig.Slot0.kI = k_DRIVE_I;
        driveMotorConfig.Slot0.kD = k_DRIVE_D;
        driveMotorConfig.Slot0.kS = k_DRIVE_FF_S;
        driveMotorConfig.Slot0.kV = k_DRIVE_FF_V;
        driveMotorConfig.Voltage.PeakForwardVoltage = 12;
        driveMotorConfig.Voltage.PeakReverseVoltage = -12;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //  Steer encoder.
        steerEncoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;

        // Pigeon 2.
        pigeon2Config.MountPose.MountPosePitch = 0;
        pigeon2Config.MountPose.MountPoseRoll = 0;
        pigeon2Config.MountPose.MountPoseYaw = 0;
    }
}
