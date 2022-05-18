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

public class TurnAndLookForTarget extends CommandBase {
  DriveTrainSubsystem driveTrain;
  private boolean isFinished = false;
  Timer timer;

  double turnConstant = 0.03;
  double min_turn = 0.01;   

  NetworkTable limelightNetworkTable;
  NetworkTableEntry tx;
  NetworkTableEntry ty;
  NetworkTableEntry ta;

  LimelightRunner limelight;

  boolean shouldTrack = true;

  /** Creates a new AutoDrive. 
   * @param driveTrain The drive train subsystem.
   */
  public TurnAndLookForTarget(DriveTrainSubsystem driveTrain) {
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

    driveTrain.arcadeDrive(0, 0);
    SmartDashboard.putNumber("ArcadeDriveY", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if (!shouldTrack)
      return;
    
    if (!limelight.getIsTracking())
    {
      System.out.println("No tracking");

      LookForObject();
      return;
    }

    double error = limelight.getX();
    //System.out.println("erorr: " + error);

    double steering_adjust = 0.0;

    if (error > 1.0)
    {
      steering_adjust = turnConstant * error - min_turn;
    }
    else if (error < 1.0)
    {
      steering_adjust = turnConstant * error + min_turn;
    }

    if (Math.abs(error) < 2 && limelight.getIsTracking())
    {
        isFinished = true;
        driveTrain.stopMotors();
        System.out.println("Aligned with target");
    }

    //System.out.println("Steering adjust: " + steering_adjust);
    driveTrain.arcadeDrive(0.0, steering_adjust);
  }

  void LookForObject()
  {
    driveTrain.tankDriveVolts(1.0, -1.0);
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