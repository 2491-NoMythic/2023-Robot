package frc.robot.settings;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeState {
    public IntakeState intakeState = null;
    private static intakeMode intakeMode;
    
    public enum intakeMode {
        CUBE,
        CONE_GROUND,
        CONE_RAMP,
        CONE_SHELF
    }


    private IntakeState(){
    
    }


    public IntakeState getInstance(){
        if (this.intakeState == null) {
            this.intakeState = new IntakeState();
            }
            return this.intakeState;
        }

    public static Command setIntakeMode(intakeMode mode){
        intakeMode = mode;
        return null;
    }

    public intakeMode getIntakeMode(){
        return intakeMode;
    }
}
