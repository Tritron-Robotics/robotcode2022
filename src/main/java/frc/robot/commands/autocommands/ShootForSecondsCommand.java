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
     * Command that makes the robot shoot for seconds.
     * 
     * @param driveTrainSubsystem The drive train subsystem.
     * @param shootingSubsystem The subsystem for shooting.
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
        isFinished = false;
        
        timer.reset();
        timer.start();
        
        driveTrain.tankDriveVolts(0, 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < seconds) 
        {
            shootingSubsystem.spinIntake(1.0);
            shootingSubsystem.fastShoot(1.0);
        } 
        else
        {
            isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveTrain.stopMotors();
        shootingSubsystem.stopAllMotors();
        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
