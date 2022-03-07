package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class IntakeForSecondsCommand extends CommandBase {
    ShootingSubsystem shootingSubsystem;

    private boolean isFinished = false;
    Timer timer;

    double seconds;
    int direction;
    /**
     * Creates a new AutoDrive.
     * 
     * @param driveTrainSubsystem The drive train subsystem.
     */
    public IntakeForSecondsCommand(double seconds, int direction, ShootingSubsystem shootingSubsystem) {
        this.shootingSubsystem = shootingSubsystem;
        this.seconds = seconds;

        timer = new Timer();
        addRequirements(shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Start intaking");
        isFinished = false;
        
        timer.reset();
        timer.start();

        shootingSubsystem.stopAllMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < seconds) 
        {
            System.out.println("intake");

            shootingSubsystem.spinIntake(-1);
        } else
        {
            isFinished = true;
            shootingSubsystem.stopAllMotors();
            System.out.println("Finished intaking");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.stopAllMotors();
        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
