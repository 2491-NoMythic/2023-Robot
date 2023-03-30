package frc.robot.settings;

public class IntakeState {
    public IntakeState intakeState = null;
    private static IntakeMode intakeMode;
    
    public enum IntakeMode {
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

    public static void setIntakeMode(IntakeMode mode){
        intakeMode = mode;
    }

    public IntakeMode getIntakeMode(){
        return intakeMode;
    }
    public boolean isConeMode() {
        return (intakeMode.equals(intakeMode.CONE_GROUND) ||
                intakeMode.equals(intakeMode.CONE_RAMP) ||
                intakeMode.equals(intakeMode.CONE_SHELF));
    }
    public boolean isCubeMode() {
        return (intakeMode.equals(intakeMode.CUBE));
    }
}
