package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class LimelightmotorSubsystem extends SubsystemBase {
     
    TalonSRX limelightMotor;
    Limelight testLimelight;

    public LimelightmotorSubsystem(){

        testLimelight = Limelight.getInstance();
        limelightMotor = new TalonSRX(2);
    }
    
    public void runLimelightmotor(double speed){ //double speed
        limelightMotor.set(ControlMode.PercentOutput, speed);
    }


}