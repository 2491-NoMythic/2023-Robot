package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.settings.LimelightValues;
    
public class Limelight {

    NetworkTableInstance inst;
    NetworkTable visionTable;

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
        //gets numbers periodically
        inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("limelight");
        NetworkTableEntry tx = visionTable.getEntry("tx");
        NetworkTableEntry ty = visionTable.getEntry("ty");
        NetworkTableEntry ta = visionTable.getEntry("ta");
        //Makes Local Variables
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        return new LimelightValues(x, y, area);
    }



    public double getXValue(){
        NetworkTableEntry tx = visionTable.getEntry("tx");
        return tx.getDouble(0.0);
    }

}
