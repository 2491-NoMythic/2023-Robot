package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.subsystems.Limelight;
public class Limelightmotor {
     
    TalonFX Limelightmotor;
    Limelight testLimelight;
    public Limelightmotor(){
        this.testLimelight = testLimelight;
   
        Limelightmotor = new TalonFX(0);
    }
    public void runLimelightmotor(){ //double speed
        Limelightmotor.set(ControlMode.PercentOutput, testLimelight.getXValue());
    }
}
