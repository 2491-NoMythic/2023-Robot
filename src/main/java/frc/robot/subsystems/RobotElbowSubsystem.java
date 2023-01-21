package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class RobotElbowSubsystem extends SubsystemBase{
    TalonFX armElbowMotor;
    
    public RobotElbowSubsystem(){
        armElbowMotor = new TalonFX(ARM_ELBOW_MOTOR_ID);
    }
    public void setElbowPower(double power){
        armElbowMotor.set(ControlMode.PercentOutput, power);
    }
}
