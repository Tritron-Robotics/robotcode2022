package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

public class IntakeForSecondsCommand extends CommandBase {
    ShootingSubsystem shootingSubsystem;

    private boolean isFinished = false;
    Timer timer;

    double seconds;
    int direction;
    /**
     * Creates the Intake for seconds command.
     * @param seconds The amount of time to intake.
     * @param direction The direction to intake for.
     * @param shootingSubsystem The shooting subsystem.
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
            shootingSubsystem.spinIntake(direction);
        } else
        {
            isFinished = true;
            shootingSubsystem.stopAllMotors();
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
