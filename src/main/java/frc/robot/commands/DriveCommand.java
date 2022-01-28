// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveCommand extends CommandBase {

  public DriveTrain driveTrain;

  public DoubleSupplier leftSpeed;
  public DoubleSupplier rightSpeed;

  /**
   * Constructor for the DriveCommand class
   * @param subsystem Subsystem for drive train
   * @param leftInput Left motors input
   * @param rightInput Right motors input
   */
  public DriveCommand(DriveTrain subsystem, DoubleSupplier leftInput, DoubleSupplier rightInput) {
    driveTrain = subsystem;
    this.leftSpeed = leftInput;
    this.rightSpeed = rightInput;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.tankDriveVolts(0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tankDrive();
  }

  /**
   * Controls the motors in the tank drive style 
   */
  private void tankDrive(){
    driveTrain.tankDriveVolts(leftSpeed.getAsDouble() * 6.0, rightSpeed.getAsDouble() * 6.0);
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
