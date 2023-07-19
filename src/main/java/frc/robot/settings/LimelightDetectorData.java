// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.Results;

/**Contains the data from the Limelight Neural network cone/cube detection pipeline*/
public class LimelightDetectorData {
    LimelightHelpers.Results llresults;
    public boolean isResultValid;
    /**Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees) */
    public double tx;
    /**Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees) */
    public double ty;
    /**Target Area (0% of image to 100% of image) */
    public double ta;
    /**0=Cone, 1=Cube, 2=merge? */
    public double classID;

    public LimelightDetectorData(Results llresults) {
        this.llresults = llresults;
        this.isResultValid = llresults.valid;
        
        if (isResultValid) {
            this.tx = llresults.targets_Detector[0].tx;
            this.ty = llresults.targets_Detector[0].ty;
            this.ta = llresults.targets_Detector[0].ta;
            this.classID = llresults.targets_Detector[0].classID;
        }
    }
}
