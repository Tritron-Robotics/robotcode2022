// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDriveCommand extends CommandBase {

  private DriveTrain driveTrain;

  private DoubleSupplier forwardInput;
  private DoubleSupplier rotInput;
  private BooleanSupplier speedMofidier;

  /**
   * Constructor for the DriveCommand class
   * @param subsystem Subsystem for drive train
   * @param forwardInput Left motors input
   * @param rotInput Right motors input
   * @param speedModifierInput Speed mofifier input. If this boolean is true, the speed of the motors will change.
   */
  public ArcadeDriveCommand(DriveTrain subsystem, DoubleSupplier forwardInput, DoubleSupplier rotInput, BooleanSupplier speedModifierInput) {
    driveTrain = subsystem;
    this.forwardInput = forwardInput;
    this.rotInput = rotInput;
    this.speedMofidier = speedModifierInput;
    
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.arcadeDrive(0, 0);
    //driveTrain.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("Execute arcade drive");
    arcadeMode();
  }
  
  /**
   * Controls the motors in arcade drive style
   */
  void arcadeMode()
  {
    double volts = speedMofidier.getAsBoolean() ? Constants.Kinematics.arcadeDriveSpeedModifierVolts : Constants.Kinematics.arcadeDriveVolts;
    
    //driveTrain.arcadeDrive(forwardInput.getAsDouble() * multiplier, rotInput.getAsDouble() * multiplier);
    driveTrain.arcadeDrive(forwardInput.getAsDouble() * volts, rotInput.getAsDouble() * volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
