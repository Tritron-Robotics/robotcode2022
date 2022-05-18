package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autocommands.DriveForwardDistance;
import frc.robot.commands.autocommands.TurnAndLookForTarget;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

/**
 * The SequentialCommandGroup for the main autonomous command
 * All of the commands that are executed during autonomous are here
 */
public class BullMode extends SequentialCommandGroup {
    
   /**
   * Creates AutonomousCommand
   * 
   * @param driveTrainSubsystem The drive train subsystem.
   * @param shootingSubsystem The shootoing subsystem
   */
    public BullMode(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) 
    {   
        addCommands(
            new TurnAndLookForTarget(driveTrainSubsystem),
            new DriveForwardDistance(driveTrainSubsystem, 10.0, 2.0)
        );

        addRequirements(driveTrainSubsystem, shootingSubsystem);
    }
}
