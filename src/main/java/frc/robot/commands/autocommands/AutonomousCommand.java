package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
    
    public AutonomousCommand(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) 
    {
        System.out.println("Autonomous command");
        // drive forward and look for ball
        addCommands(
            //new DriveForwardDistance(driveTrainSubsystem, 5.0, 1.0)
            new PickUpBall(4.0, driveTrainSubsystem, shootingSubsystem),
            new RotateDegrees(180, driveTrainSubsystem),
            new DriveForwardSeconds(2.0, 1, driveTrainSubsystem),
            new IntakeForSecondsCommand(0.25, -1, shootingSubsystem),
            new ShootForSecondsCommand(1.0, driveTrainSubsystem, shootingSubsystem),
            // new JitterBugSequentialCommand(driveTrainSubsystem, shootingSubsystem),
            // new RandomCommands(driveTrainSubsystem, shootingSubsystem),
            new DoAbsolutelyNothingForSeconds(10.0, driveTrainSubsystem, shootingSubsystem)
        );

        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }
}
