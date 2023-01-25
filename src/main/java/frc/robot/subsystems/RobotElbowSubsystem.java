package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class RobotElbowSubsystem extends SubsystemBase{
    TalonSRX armElbowMotor;
    
    public RobotElbowSubsystem(){
        armElbowMotor = new TalonSRX(ARM_ELBOW_MOTOR_ID);
    }
    public void setElbowPower(double power){
        armElbowMotor.set(ControlMode.PercentOutput, power);
    }
    public void setBrakeMode() {
        armElbowMotor.setNeutralMode(NeutralMode.Brake);
    }

}
