package frc.robot.settings;

public class IntakeState {
    private static IntakeState intakeState = null;
    private static IntakeMode intakeMode;
    
    public enum IntakeMode {
        CUBE,
        CONE_GROUND,
        CONE_RAMP,
        CONE_SHELF,
        CUBE_SHELF,
    }


    private IntakeState(){
        intakeMode = IntakeMode.CUBE;
    }


    public static IntakeState getInstance(){
        if (intakeState == null) {
            intakeState = new IntakeState();
            }
            return intakeState;
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
        return (intakeMode.equals(intakeMode.CUBE) ||
                intakeMode.equals(intakeMode.CUBE_SHELF));
    }
}
