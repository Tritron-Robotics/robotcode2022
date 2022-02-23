package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;

public class SequentialCommand extends SequentialCommandGroup {
    public SequentialCommand(DriveTrainSubsystem driveTrainSubsystem)
    {
        TrackObjectCommand autoDriveCommand = new TrackObjectCommand(driveTrainSubsystem);
        addCommands(
        // Drive forward the specified distance
        autoDriveCommand,
        new BullCommand(driveTrainSubsystem, autoDriveCommand));
    }
}
