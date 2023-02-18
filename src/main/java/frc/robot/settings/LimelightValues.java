// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.LimelightHelpers;

/** Add your docs here. */
public class LimelightValues {
        public boolean isResultValid;
        int numTags;
        double[] tx = new double[5];
        double[] ty = new double[5];
        double[] ta = new double[5];
        Pose2d botPoseRed;
        Pose2d botPoseBlue;
        double timestamp_LIMELIGHT_publish;
        double timestamp_RIOFPGA_capture;
        double timestamp_Latency_capture;
        double timestamp_Latency_pipeline;
        public LimelightValues(LimelightHelpers.LimelightResults llresults){
            this.isResultValid = llresults.targetingResults.valid;
            if (isResultValid) {
                this.numTags = llresults.targetingResults.targets_Fiducials.length;
                for (int i = 0; i < numTags; i++) {
                    this.tx[i] = llresults.targetingResults.targets_Fiducials[i].tx;
                    this.ty[i] = llresults.targetingResults.targets_Fiducials[i].ty;
                    this.ta[i] = llresults.targetingResults.targets_Fiducials[i].ta;
                }
                
                this.botPoseRed = llresults.targetingResults.getBotPose2d_wpiRed();
                this.botPoseBlue = llresults.targetingResults.getBotPose2d_wpiBlue();
                this.timestamp_LIMELIGHT_publish = llresults.targetingResults.timestamp_LIMELIGHT_publish;
                this.timestamp_RIOFPGA_capture = llresults.targetingResults.timestamp_RIOFPGA_capture;
                this.timestamp_Latency_capture = llresults.targetingResults.latency_capture;
                this.timestamp_Latency_pipeline = llresults.targetingResults.latency_pipeline;
            }
        }
        public double gettx(int index){return tx[index];}
        public double getty(int index){return ty[index];}
        public double getta(int index){return ta[index];}
        public Pose2d getbotPose(){
            if(DriverStation.getAlliance() ==  Alliance.Red){
                return botPoseRed;
            }else{
                return botPoseBlue;
            }
        }
        public double[] gettimestamp(){
            return new double[] {timestamp_LIMELIGHT_publish, timestamp_RIOFPGA_capture, timestamp_Latency_capture, timestamp_Latency_pipeline};}
}
