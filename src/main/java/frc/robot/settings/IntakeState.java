package frc.robot.settings;

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

    public static void setIntakeMode(intakeMode mode){
        intakeMode = mode;
    }

    public intakeMode getIntakeMode(){
        return intakeMode;
    }
}
