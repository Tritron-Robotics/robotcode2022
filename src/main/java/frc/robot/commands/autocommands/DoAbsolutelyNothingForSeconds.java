package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class DoAbsolutelyNothingForSeconds extends CommandBase {
    ShootingSubsystem shootingSubsystem;
    DriveTrainSubsystem dts;


    private boolean isFinished = false;
    Timer timer;

    double seconds;
    /**
     * Creates a new AutoDrive.
     * 
     * @param driveTrainSubsystem The drive train subsystem.
     */
    public DoAbsolutelyNothingForSeconds(double seconds, DriveTrainSubsystem dts, ShootingSubsystem shootingSubsystem) {
        this.shootingSubsystem = shootingSubsystem;
        this.dts = dts;
        this.seconds = seconds;

        timer = new Timer();
        addRequirements(shootingSubsystem, dts);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        System.out.println("Start waiting to do absolutely nothing");
        isFinished = false;
        
        timer.reset();
        timer.start();

        shootingSubsystem.stopAllMotors();
        dts.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.get() < seconds) 
        {
            System.out.println("Doing nothing right now.");
            shootingSubsystem.stopAllMotors();
            dts.stopMotors();
        } else
        {
            isFinished = true;
            System.out.println("Finished doing nothing");
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shootingSubsystem.stopAllMotors();
        dts.stopMotors();
        isFinished = true;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
