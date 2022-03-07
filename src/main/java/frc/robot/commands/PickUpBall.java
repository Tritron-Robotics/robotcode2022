package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class PickUpBall extends CommandBase {
  DriveTrainSubsystem driveTrain;
  ShootingSubsystem shootingSubsystem;

  private boolean isFinished = false;
  Timer timer;

  /**
   * Creates a new AutoDrive.
   * 
   * @param driveTrainSubsystem The drive train subsystem.
   */
  public PickUpBall(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) {
    this.driveTrain = driveTrainSubsystem;
    this.shootingSubsystem = shootingSubsystem;

    timer = new Timer();
    addRequirements(driveTrainSubsystem, shootingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Start picking up ball");
    isFinished = false;
    timer.reset();
    timer.start();

    driveTrain.arcadeDrive(0, 0);
    // SmartDashboard.putNumber("ArcadeDriveY", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.get() < 8.0) {
      MoveForward();
      shootingSubsystem.spinIntake(0.7);
      System.out.println("Picking up ball for: " + timer.get() + "secs");
    }
    else
    {
      isFinished = true;
    }
  }

  void MoveForward() {
    driveTrain.tankDriveVolts(1.0, 1.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    shootingSubsystem.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
