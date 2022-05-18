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

        //addCommands(
            
            //new ShootForSecondsCommand(5.0, false, driveTrainSubsystem, shootingSubsystem),
            //new DriveForwardDistance(driveTrainSubsystem, 9.0, -2.0)
        //    );

        // addCommands(
        //     new DriveForwardSeconds(0.1, 1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, -1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, 1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, -1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, 1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, -1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, 1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, -1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, 1, driveTrainSubsystem),
        //     new DriveForwardSeconds(0.1, -1, driveTrainSubsystem),
        //     new RotateDegrees(180, driveTrainSubsystem, 2.0),
        //     new DoAbsolutelyNothingForSeconds(0.25, driveTrainSubsystem, shootingSubsystem),
        //     new RotateDegrees(180, driveTrainSubsystem, 2.0),
        //     new ShootForSecondsCommand(1.0, true, driveTrainSubsystem, shootingSubsystem)
        // );


        // addCommands(
        //     new DriveForwardDistance(driveTrainSubsystem, 3.7, -2.0),
        //     //new IntakeForSecondsCommand(2.0, -1, shootingSubsystem),
        //     new ShootForSecondsCommand(5.0, true, driveTrainSubsystem, shootingSubsystem),
        //     new DriveForwardDistance(driveTrainSubsystem, 3.0, -1.0),
        //     new RotateDegrees(150, driveTrainSubsystem, 3.0),
        //     new DoAbsolutelyNothingForSeconds(10.0, driveTrainSubsystem, shootingSubsystem)  
        // );
        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }
}
