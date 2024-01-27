package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private Pose2d m_limelightPose;
    NetworkTable table;
    NetworkTableEntry tx, ty, ta;

    public Vision() {
    table  = NetworkTableInstance.getDefault().getTable("limelight");
    
     tx = table.getEntry("tx");
     ty = table.getEntry("ty");
     ta = table.getEntry("ta");
        
    }

    private double Latency;

    public double getLatency() {
    return Latency;

    }

    public Pose2d poseupdate() 
    {
        return m_limelightPose;
    }
    

    @Override
    public void periodic() {

        
        double[] botposeValue = new double[7];
        table.getEntry("botpose").getDoubleArray(botposeValue);

        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);
        
        SmartDashboard.putNumber("LimelightX", botposeValue[0]);
        SmartDashboard.putNumber("LimelightY", botposeValue[1]);
        SmartDashboard.putNumber("LimelightArea", area); 
        m_limelightPose = new Pose2d(botposeValue[0], botposeValue[1], new Rotation2d(botposeValue[5]));
        //To-do: Verify
        Latency = botposeValue[6];
    }

}
