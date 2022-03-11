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
            new PickUpBall(3.5, driveTrainSubsystem, shootingSubsystem),
            new RotateDegrees(180.0, driveTrainSubsystem, 2.0),
            new DriveForwardDistance(driveTrainSubsystem, 3.8, 2.0),
            new RotateDegrees(10.0, driveTrainSubsystem, 1.0),
            new IntakeForSecondsCommand(0.25, -1, shootingSubsystem),
            new ShootForSecondsCommand(1.0, driveTrainSubsystem, shootingSubsystem),
            new DriveForwardDistance(driveTrainSubsystem, 4.0, -2.0),
            //new RandomCommands(driveTrainSubsystem, shootingSubsystem),
            new DoAbsolutelyNothingForSeconds(60.0, driveTrainSubsystem, shootingSubsystem)
        );

        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }
}
