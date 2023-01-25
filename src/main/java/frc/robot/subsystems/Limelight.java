package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
    
public class Limelight {
    NetworkTableEntry tx, ty, ta;
    NetworkTableInstance inst;
    NetworkTable visionTable;
    public void putThingOnDashboard(){
        //gets numbers periodically
        inst = NetworkTableInstance.getDefault();
        visionTable = inst.getTable("limelight");
        NetworkTableEntry tx = visionTable.getEntry("tx");
        NetworkTableEntry ty = visionTable.getEntry("ty");
        NetworkTableEntry ta = visionTable.getEntry("ta");
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
    public double getXValue(){
        NetworkTableEntry tx = visionTable.getEntry("tx");
        return tx.getDouble(0.0);
    }
}
