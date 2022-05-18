// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightRunner;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AdjustDistanceFromTarget extends CommandBase {
  DriveTrainSubsystem driveTrain;
  private boolean isFinished = false;
  Timer timer = new Timer();

  double moveConstant = 0.03;
  double minMove = 0.01;   

  NetworkTable limelightNetworkTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  LimelightRunner limelight;

  boolean shouldTrack = true;

  boolean isPaused = false;
  Timer globalTimer = new Timer();

  /** Creates a new AutoDrive. 
   * @param driveTrain The drive train subsystem.
   */
  public AdjustDistanceFromTarget(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;
    limelight = LimelightRunner.getInstance();

    timer = new Timer();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    globalTimer.start();

    driveTrain.arcadeDrive(0, 0);
    SmartDashboard.putNumber("ArcadeDriveY", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (!shouldTrack)
      return;
    
    
    // if (justMoveForward)
    // {
    //     if (timer.get() < 2.0)
    //     {
    //         System.out.println("Move forward");
    //         driveTrain.tankDriveVolts(1.0, 1.0);
    //     } 
    //     else
    //     {
    //         isFinished = true;
    //         System.out.println("Finished moving forward");
    //     }
    //     return;
    // }
    if (!limelight.getIsTracking())
    {
      timer.stop();
      isPaused = true;
      System.out.println("No tracking");
      driveTrain.tankDriveVolts(1.0, 1.0);
      return;
    }

    double error = limelight.getY();
    //System.out.println("erorr: " + error);

    double movementAdjust = 0.0;

    if (isPaused)
    {
        timer.start();
        isPaused= false;
    }
    if (error > 1.0)
    {
      movementAdjust = moveConstant * error - minMove;
    }
    else if (error < 1.0)
    {
      movementAdjust = moveConstant * error + minMove;
    }

    if (globalTimer.get() > 4.0)
    {
        if (globalTimer.get() < 5.0)
        {
            driveTrain.tankDriveVolts(1.0, 1.0);
            System.out.println("exceeded time limits move forward");
            
        } 
        else if (globalTimer.get() > 5.0)
        {
            isFinished = true;
            System.out.println("finished global timer");
        }
    }

    if (Math.abs(error) < 2 && limelight.getIsTracking())
    {
        isFinished = true;
        driveTrain.stopMotors();
        System.out.println("Aligned with target");
    }

    System.out.println("Movement adjust: " + movementAdjust);
    driveTrain.arcadeDrive(movementAdjust, 0.0);
  }

  public void SetShouldTrack(boolean shouldTrack)
  {
    this.shouldTrack = shouldTrack;
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
