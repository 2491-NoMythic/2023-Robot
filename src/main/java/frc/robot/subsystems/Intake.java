package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.Intake.*;

public class Intake extends SubsystemBase{
    insert_motor tempIntakeMotor;

    public Intake(){
            tempIntakeMotor = new insert_motor(INTAKE_MOTOR_ID);
    }
    public void intakeMotor(double speed){
        tempIntakeMotor.set(speed);
    }
}
