package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

/**
 * This class does absolutely nothing. 
 * It just wastes time
 */
public class DoAbsolutelyNothingForSeconds extends CommandBase {
    ShootingSubsystem shootingSubsystem;
    DriveTrainSubsystem driveTrainSubsystem;

    private boolean isFinished = false;
    Timer timer;

    double seconds;
    /**
     * Creates DoAbsolutelyNothingForSeconds
     * @param seconds The amount of time to do absolutely nothing 
     * @param driveTrainSubsystem The drive train subsystem.
     * @param shootingSubsystem The shooting subsystem.
     */
    public DoAbsolutelyNothingForSeconds(double seconds, DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) {
        this.shootingSubsystem = shootingSubsystem;
        this.driveTrainSubsystem = driveTrainSubsystem;
        this.seconds = seconds;

        timer = new Timer();
        addRequirements(shootingSubsystem, driveTrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        
        timer.reset();
        timer.start();

        shootingSubsystem.stopAllMotors();
        driveTrainSubsystem.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < seconds) 
        {
            shootingSubsystem.stopAllMotors();
            driveTrainSubsystem.stopMotors();
        } else
        {
            isFinished = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.stopAllMotors();
        driveTrainSubsystem.stopMotors();
        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
