package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class DriveForwardSeconds extends CommandBase {
  DriveTrainSubsystem driveTrain;
  ShootingSubsystem shootingSubsystem;

  private boolean isFinished = false;

  double seconds;
  double dir;
  Timer timer;

  /**
   * Creates a new AutoDrive.
   * 
   * @param driveTrainSubsystem The drive train subsystem.
   */
  public DriveForwardSeconds(double seconds, double dir, DriveTrainSubsystem driveTrainSubsystem) {
    this.driveTrain = driveTrainSubsystem;
    this.seconds = seconds;
    this.dir = dir;
    timer = new Timer();
    addRequirements(driveTrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    timer.reset();
    timer.start();

    driveTrain.arcadeDrive(0, 0);
    // SmartDashboard.putNumber("ArcadeDriveY", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < seconds) {
      MoveForward();
      System.out.println("moving forward for: " + timer.get() + "secs");
    }
    else
    {
      isFinished = true;
    }
  }

  void MoveForward() {
    driveTrain.tankDriveVolts(2.0 * dir, 2.0 * dir);
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
