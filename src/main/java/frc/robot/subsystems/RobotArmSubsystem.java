package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotArmSubsystem extends SubsystemBase{
    TalonSRX armShoulderMotor;
    TalonSRX armElbowMotor;
    
    public RobotArmSubsystem(){
        armElbowMotor = new TalonSRX(ARM_ELBOW_MOTOR_ID);
        armShoulderMotor = new TalonSRX(ARM_SHOULDER_MOTOR_ID);
    }
    public void setElbowPower(double power){
        armElbowMotor.set(ControlMode.PercentOutput, power);
    }
    public void setShoulderPower(double power){
        armShoulderMotor.set(ControlMode.PercentOutput, power);
    }
    public void setBrakeMode() {
        armShoulderMotor.setNeutralMode(NeutralMode.Brake);
        armElbowMotor.setNeutralMode(NeutralMode.Brake);
    }
}
