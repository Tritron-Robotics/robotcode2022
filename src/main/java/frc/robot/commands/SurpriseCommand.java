// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SurpriseCommand extends CommandBase {
// Declare Subsystems and other stuffies.
  DriveTrain driveTrain;
  private boolean isFinished = false;
  Timer timer;

  DoubleSupplier testInput;

  NetworkTable limelightNetworkTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;
  NetworkTableEntry tv;

  private static final int buffer = 4;

  double x, y, area;
  boolean isTrackingObject;
  /** Creates a new AutoDrive. 
   * @param driveTrain The drive train subsystem.
   */
  public SurpriseCommand(DriveTrain driveTrain) {
    this.driveTrain = driveTrain;
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightNetworkTable.getEntry("tx");
    ty = limelightNetworkTable.getEntry("ty");
    tv = limelightNetworkTable.getEntry("tv");
    ta = limelightNetworkTable.getEntry("ta");  

    timer = new Timer();

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    driveTrain.arcadeDrive(0, 0);
    SmartDashboard.putNumber("ArcadeDriveY", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    getLimelightData();

    if (isTrackingObject)
    {
        if (timer.get() == 0.0)
        {
            timer.start();
            System.out.println("timer == 0");
        }      
    } else {
        timer.reset();
        System.out.println("not tracking object");
        findObjectToTrack();
    }

    // if is tracking object for more than 0.5 seconds
    if (timer.get() > 0.0 && timer.get() < 0.5)
    {         
        System.out.println("Time: " + timer.get());
        if (timer.get() > 0.5 && timer.get() < 3.0)
        {
            driveTrain.arcadeDrive(0.3, 0);
            System.out.println("forward");
        } else if  (timer.get() > 3.0 && timer.get() <= 3.33)
        {
            driveTrain.arcadeDrive(-0.3, 0);  
            System.out.println("backward");
        } else if (timer.get() > 3.33 && timer.get() <= 3.66){
            driveTrain.arcadeDrive(0.3, 0);
            System.out.println("forward");

        } else if (timer.get() > 3.66 && timer.get() <= 4.0)
        {
            driveTrain.arcadeDrive(-0.3, 0);
            System.out.println("backward");

        }  else if (timer.get() > 4.5 && timer.get() <= 5.5)
        {
            driveTrain.arcadeDrive(0.5, 0);
            System.out.println("CHAAAARGE!!!!");

        }  else if (timer.get() >= 6.5)
        {
            System.out.println("Done charging");

        } else {
            System.out.println("Resting");
        }
        
    } else {
        rotateToTrackedObject();
    }
    
    // if (timer.get() < 5.0)
    // {
    // //driveTrain.arcadeDrive(0.3, 0);
    // } else {
    //     isFinished = true;
    // }
    //System.out.println("execute surprise");
  }

  void getLimelightData()
  {
    //read values periodically
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    isTrackingObject = tv.getDouble(0.0) == 1.0 ? true : false;

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putBoolean("IsTrackingObject", isTrackingObject);

  }

  void rotateToTrackedObject()
  {
    if (x > buffer)
    {
      driveTrain.arcadeDrive(0, 0.3);
      //System.out.println("Rotate clockwise");
    } 
    else if (x < -buffer)
    {
      driveTrain.arcadeDrive(0, -0.3);
      //System.out.println("Rotate counter-clockwise");
    }
  }

  void findObjectToTrack()
  {
    driveTrain.arcadeDrive(0, 0.3);
  }

  void CHARGE()
  {
    driveTrain.arcadeDrive(0.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
