package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class ArmSubsystem extends SubsystemBase{
    TalonFX armShoulderMotor;
    TalonFX armElbowMotor;
    
    public ArmSubsystem(){
        armShoulderMotor = new TalonFX(ARM_SHOULDER_MOTOR_ID);
        armElbowMotor = new TalonFX(ARM_ELBOW_MOTOR_ID);
    }
    public void armShoulderMotor(){ //double speed
        armShoulderMotor.set(ControlMode.PercentOutput,0.5);//set to speed later
    }
    public void armElbowMotor(){ //double speed
        armElbowMotor.set(ControlMode.PercentOutput,0.5);//set to speed later
    }
}
