package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Intake.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class Intake extends SubsystemBase{
    TalonFX tempIntakeMotor;

    public Intake(){
            tempIntakeMotor = new TalonFX(INTAKE_MOTOR_ID);
    }
    public void intakeMotor(){ //double speed
        tempIntakeMotor.set(ControlMode.PercentOutput,0.5);//set to speed later
    }
}
