// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class ArcadeDriveCommand extends CommandBase {

  private DriveTrainSubsystem driveTrain;

  private DoubleSupplier forwardInput;
  private DoubleSupplier rotInput;
  private BooleanSupplier partyModeInput;
  private BooleanSupplier precisionModeInput;


  /**
   * Constructor for the ArcadeDriveCommand class
   * @param subsystem The DriveTrain subsystem
   * @param forwardInput Forward input. Positive value will move the robot forward.
   * @param rotInput Rotation input. Positive values will rotate the robot clockwise.
   * @param partyModeInput Speed mofifier input. If this boolean is true, the speed of the motors will change.
   */
  public ArcadeDriveCommand(DriveTrainSubsystem subsystem, DoubleSupplier forwardInput, DoubleSupplier rotInput, BooleanSupplier partyModeInput, BooleanSupplier precisionModeInput) {
    driveTrain = subsystem;
    this.forwardInput = forwardInput;
    this.rotInput = rotInput;
    this.partyModeInput = partyModeInput;
    this.precisionModeInput = precisionModeInput;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.arcadeDrive(0, 0);
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
    double volts = Constants.Kinematics.arcadeDriveVoltage;
    if (partyModeInput.getAsBoolean())
    {
      volts = Constants.Kinematics.arcadeDrivePartyModeValue;
    } 
    else if (precisionModeInput.getAsBoolean())
    {
      volts = Constants.Kinematics.arcadeDrivePrecisionMode;
    }
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
