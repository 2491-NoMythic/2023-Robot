package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Arm.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class RobotArmSubsystem extends SubsystemBase{
    CANSparkMax armShoulderMotor;
    CANSparkMax armElbowMotor;
    RelativeEncoder armShoulderEncoder;
    RelativeEncoder armElbowEncoder;
    
    public RobotArmSubsystem(){
        armElbowMotor = new CANSparkMax(ARM_ELBOW_MOTOR_ID, MotorType.kBrushless);
        armShoulderMotor = new CANSparkMax(ARM_SHOULDER_MOTOR_ID, MotorType.kBrushless);
        armElbowEncoder = armElbowMotor.getEncoder();
        armShoulderEncoder = armShoulderMotor.getEncoder();
    }
    public void setElbowPower(double power){
        armElbowMotor.set(power);
    }
    public void setShoulderPower(double power){
        armShoulderMotor.set(power);
    }
    
    public void resetElbowEncoder(){
        armElbowEncoder.setPosition(0);
    }

    public void resetShoulderEncoder(){
        armShoulderEncoder.setPosition(0);
    }
    
    public double getElbowEncoder(){
        return armElbowEncoder.getPosition();
    }

    public double getShoulderEncoder(){
        return armShoulderEncoder.getPosition();
    }



    public void setBrakeMode() {
        armShoulderMotor.setIdleMode(IdleMode.kBrake);
        armElbowMotor.setIdleMode(IdleMode.kBrake);
    }
}
