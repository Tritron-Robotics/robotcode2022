package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class AutonomousCommand extends SequentialCommandGroup {
    
    public AutonomousCommand(DriveTrainSubsystem driveTrainSub, ShootingSubsystem shootingSub) 
    {
        System.out.println("Autonomous command group");
        // drive forward and look for ball
        addCommands(
            // new LookForBall(driveTrainSub),
            new PickUpBall(driveTrainSub, shootingSub),
            new RotateDegrees(180, driveTrainSub)
            //new ShootForSecondsCommand(2.0, driveTrainSub, shootingSub)
        );

        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
