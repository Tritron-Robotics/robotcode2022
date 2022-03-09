package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class DriveForwardDistance extends CommandBase {
    DriveTrainSubsystem driveTrainSub;
    double distance;
    double speed;

    boolean isFinished = false;

    double startingMotorDistance;


    public DriveForwardDistance(DriveTrainSubsystem subsystem, double distance, double speed)
    {
        this.driveTrainSub = subsystem;
        this.distance = distance;
        this.speed = speed;
    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize drive forward distance");
    driveTrainSub.stopMotors();
    startingMotorDistance = driveTrainSub.getAverageEncoderDistanceInFeet();
    System.out.println("Start : " + startingMotorDistance);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    double distance = Math.abs(driveTrainSub.getAverageEncoderDistanceInFeet() - startingMotorDistance);
    System.out.println("Distance: " + distance);
    driveTrainSub.tankDriveVolts(speed, speed);

    if (distance > 8)
    {
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("End forward distance + " + interrupted);
    driveTrainSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
