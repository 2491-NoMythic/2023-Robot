package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class RobotShoulderSubsystem extends SubsystemBase{
    TalonFX armShoulderMotor;
    
    public RobotShoulderSubsystem(){
        armShoulderMotor = new TalonFX(ARM_SHOULDER_MOTOR_ID);
    }
    public void setShoulderPower(double power){
        armShoulderMotor.set(ControlMode.PercentOutput, power);
    }
}
