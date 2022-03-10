package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

/**
 * The SequentialCommandGroup for the main autonomous command
 * All of the commands that are executed during autonomous are here
 */
public class AutonomousCommand extends SequentialCommandGroup {
    
   /**
   * Creates AutonomousCommand
   * 
   * @param driveTrainSubsystem The drive train subsystem.
   * @param shootingSubsystem The shootoing subsystem
   */
    public AutonomousCommand(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) 
    {
        // drive forward and look for ball
        addCommands(
            new PickUpBall(4.0, driveTrainSubsystem, shootingSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, 2.0),
            new DriveForwardSeconds(2.0, 1, driveTrainSubsystem),
            new IntakeForSecondsCommand(0.25, -1, shootingSubsystem),
            new ShootForSecondsCommand(1.0, driveTrainSubsystem, shootingSubsystem),
            new RandomCommands(driveTrainSubsystem, shootingSubsystem),
            new DoAbsolutelyNothingForSeconds(10.0, driveTrainSubsystem, shootingSubsystem)
        );

        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }
}
