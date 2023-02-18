package frc.robot.subsystems;


import frc.robot.LimelightHelpers;
import frc.robot.settings.LimelightValues;
public class Limelight {

    private static Limelight limelight;

    private void Limelight(){
    }
    
    public static Limelight getInstance(){
        if (limelight == null){
            limelight = new Limelight();
        }
        return limelight;
    }
    public LimelightValues getLimelightValues(){
        return new LimelightValues(LimelightHelpers.getLatestResults(""));
    }
}
