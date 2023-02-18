package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotArmSubsystem extends SubsystemBase{
    CANSparkMax armShoulderMotor;
    CANSparkMax armElbowMotor;
    
    public RobotArmSubsystem(){
        armElbowMotor = new CANSparkMax(ARM_ELBOW_MOTOR_ID, MotorType.kBrushless);
        armShoulderMotor = new CANSparkMax(ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless);
    }
    public void setElbowPower(double power){
        armElbowMotor.set(power);
    }
    public void setShoulderPower(double power){
        armShoulderMotor.set(power);
    }
    public void setBrakeMode() {
        armShoulderMotor.setIdleMode(IdleMode.kBrake);
        armElbowMotor.setIdleMode(IdleMode.kBrake);
    }
}
