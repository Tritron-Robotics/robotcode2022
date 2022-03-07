package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveForwardDistance extends CommandBase {
    DriveTrainSubsystem driveTrainSub;
    double distance;
    double speed;

    boolean isFinished = false;

    double startingMotorPosition;

    public DriveForwardDistance(DriveTrainSubsystem subsystem, double distance, double speed)
    {
        this.driveTrainSub = subsystem;
        this.distance = distance;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrainSub.stopMotors();
    startingMotorPosition = driveTrainSub.getAverageEncoderDistance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    System.out.print("Distance: " + (driveTrainSub.getAverageEncoderDistance() - startingMotorPosition));
    driveTrainSub.tankDriveVolts(speed, speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrainSub.stopMotors();
    isFinished = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
