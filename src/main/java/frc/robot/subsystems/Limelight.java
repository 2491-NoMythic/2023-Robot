package frc.robot.subsystems;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;
import frc.robot.settings.LimelightDetectorData;
import frc.robot.settings.LimelightFiducialData;
import frc.robot.settings.Constants.Vision;
public class Limelight {

    private static Limelight limelight;

    private  Limelight(){
      vision_thread();
    }
    
    public LimelightFiducialData latestAprilTagValues;
    public LimelightDetectorData latestDetectorValues;

    public static Limelight getInstance(){
        if (limelight == null){
            limelight = new Limelight();
        }
        return limelight;
    }
    private LimelightFiducialData getAprilTagValues(){
        return new LimelightFiducialData(LimelightHelpers.getLatestResults(Vision.LIMELIGHT_APRILTAG_NAME).targetingResults, LimelightHelpers.getTV(Vision.LIMELIGHT_APRILTAG_NAME));
    }
    private LimelightDetectorData getNeuralDetectorValues(){
        return new LimelightDetectorData(LimelightHelpers.getLatestResults(Vision.LIMELIGHT_NEURAL_NAME).targetingResults, LimelightHelpers.getTV(Vision.LIMELIGHT_NEURAL_NAME));
    }
    
    public void vision_thread(){
        try{
          new Thread(() -> { // this will make a seperate thread for limelight things to happen on and not interrup other code 
            while(true){
              periodic();
              try {
                Thread.sleep(20);
              } catch (InterruptedException e) {
                e.printStackTrace();
              }
            }
          }).start();
        }catch(Exception e){}
      }
    
      public void periodic() {
        this.latestAprilTagValues = getAprilTagValues();
        this.latestDetectorValues = getNeuralDetectorValues();
      }
}
