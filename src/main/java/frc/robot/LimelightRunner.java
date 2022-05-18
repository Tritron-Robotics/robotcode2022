package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightRunner {
    
    private static LimelightRunner instance;

    NetworkTable limelightNetworkTable;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tv;

    double x, y, a;

  /**
   * Returns the Scheduler instance.
   *
   * @return the instance
   */
  public static LimelightRunner getInstance() {
    if (instance == null) {
      instance = new LimelightRunner();
    }
    return instance;
  }

  public LimelightRunner()
  {
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    //limelightNetworkTable.getEntry("ledMode").setNumber(3);
    tx = limelightNetworkTable.getEntry("tx");
    ty = limelightNetworkTable.getEntry("ty");
    ta = limelightNetworkTable.getEntry("ta");  
    tv = limelightNetworkTable.getEntry("tv"); 
  }

  public NetworkTable getNetworkTable()
  {
    return limelightNetworkTable;
  }

  public void execute()
  {  
    // System.out.println(limelightNetworkTable.getEntry("ledMode").getDouble(0));

    //limelightNetworkTable.getEntry("ledMode").setNumber(3);
    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", a);
  }

  public double getX()
  {
      x = tx.getDouble(0.0);
      return x;
  }

  public double getY()
  {
      y = ty.getDouble(0.0);
      return y;
  }

  public double getArea()
  {
    a = ta.getDouble(0.0);
    return a;
  }

  public void setLedMode(int mode)
  {
    limelightNetworkTable.getEntry("ledMode").setNumber(mode);
  }

  public boolean getIsTracking()
  {
    return (tv.getDouble(0.0) == 1.0 ? true : false);
  }

}
