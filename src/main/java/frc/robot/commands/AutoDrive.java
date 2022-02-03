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

public class AutoDrive extends CommandBase {
  // Declare Subsystems and other stuffies.
  DriveTrain driveTrain;
  private boolean isFinished = false;
  Timer timer;

  DoubleSupplier testInput;

  NetworkTable limelightNetworkTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  /** Creates a new AutoDrive. 
   * @param driveTrain The drive train subsystem.
   */
  public AutoDrive(DriveTrain driveTrain, DoubleSupplier testInput) {
    this.driveTrain = driveTrain;
    this.testInput = testInput;
    limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
    tx = limelightNetworkTable.getEntry("tx");
    ty = limelightNetworkTable.getEntry("ty");
    ta = limelightNetworkTable.getEntry("ta");

    

    timer = new Timer();
    System.out.println("Auto Drive constructor");
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    System.out.println("Initialize");
    driveTrain.arcadeDrive(0, 0);
    SmartDashboard.putNumber("ArcadeDriveY", 0);
    // while(timer.get() < 5.0) {
    //   driveTrain.arcadeDrive(testInput.getAsDouble(), 0);
    // }
    // isFinished = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    //read values periodically
    double x = tx.getDouble(0.0);
    double y = ty.getDouble(0.0);
    double area = ta.getDouble(0.0);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);


    System.out.println(x);
    if (x > 4)
    {
      driveTrain.arcadeDrive(0, 0.3);
      //System.out.println("Rotate clockwise");
    } 
    else if (x < -4)
    {
      driveTrain.arcadeDrive(0, -0.3);
      //System.out.println("Rotate counter-clockwise");

    }
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
