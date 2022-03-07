package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootForSecondsCommand extends CommandBase {
    DriveTrainSubsystem driveTrain;
    ShootingSubsystem shootingSubsystem;

    private boolean isFinished = false;
    Timer timer;

    double seconds;
    /**
     * Creates a new AutoDrive.
     * 
     * @param driveTrainSubsystem The drive train subsystem.
     */
    public ShootForSecondsCommand(double seconds, DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) {
        this.driveTrain = driveTrainSubsystem;
        this.shootingSubsystem = shootingSubsystem;
        this.seconds = seconds;

        timer = new Timer();
        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Start shooting");
        isFinished = false;
        
        timer.reset();
        timer.start();

        driveTrain.arcadeDrive(0, 0);
        // SmartDashboard.putNumber("ArcadeDriveY", 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (timer.get() < seconds) 
        {
            System.out.println("Shoot");

            shootingSubsystem.spinIntake(1.0);
            shootingSubsystem.fastShoot(1.0);
        } else
        {
            isFinished = true;
            System.out.println("Finished shooting");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.stopMotors();
        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
