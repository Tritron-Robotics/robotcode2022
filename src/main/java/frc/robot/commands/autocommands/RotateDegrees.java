package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class RotateDegrees extends CommandBase {
    DriveTrainSubsystem driveTrain;

    private boolean isFinished = false;
    Timer timer;
    double degrees;
    double degreesPerSecond = 39;

    /**
     * Creates a new AutoDrive.
     * 
     * @param driveTrainSubsystem The drive train subsystem.
     */
    public RotateDegrees(double degrees, DriveTrainSubsystem driveTrainSubsystem) {
        this.driveTrain = driveTrainSubsystem;
        this.degrees = degrees;

        timer = new Timer();
        addRequirements(driveTrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        isFinished = false;
        System.out.print("Start rotate degrees");

        timer.reset();
        timer.start();

        driveTrain.tankDriveVolts(0, 0);

        // SmartDashboard.putNumber("ArcadeDriveY", 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(degrees /  degreesPerSecond);
        if (timer.get() < degrees / degreesPerSecond) {
            System.out.println(timer.get());
            Rotate();
        } else {
            isFinished = true;
        }
    }

    void Rotate() {
        driveTrain.tankDriveVolts(1.0, -1.0);
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
