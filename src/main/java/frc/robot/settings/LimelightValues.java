// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.settings;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class LimelightValues {
            //stuff
        double tx;
        double ty;
        double ta;
        Pose2d botPose;
        public LimelightValues(double tx, double ty, double ta, Pose2d botPose){
            //set stuff equal to stuff
            this.tx = tx;
            this.ty = ty;
            this.ta = ta;
            this.botPose = botPose;

        }
        public double gettx(){return tx;}
        public double getty(){return ty;}
        public double getta(){return ta;}
        public Pose2d getbotPose(){
            if(SmartDashboard.getBoolean("isRedAlliance", true)){
                return botPose.relativeTo(new Pose2d(new Translation2d(8.25, 4), Rotation2d.fromDegrees(0)));
            }else
                return botPose.relativeTo(new Pose2d(new Translation2d(8.25, 4), Rotation2d.fromDegrees(180)));
        }

}
