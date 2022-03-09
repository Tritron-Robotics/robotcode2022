package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class RandomCommands extends SequentialCommandGroup {
    
    SequentialCommandGroup jitter;
    
    public RandomCommands(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) 
    {
        jitter = new SequentialCommandGroup(
        new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem)
        );

        // drive forward and look for ball
        addCommands(            
            new RotateDegrees(15, driveTrainSubsystem, 2.0),
            new RotateDegrees(30, driveTrainSubsystem, -2.0),
            new RotateDegrees(30, driveTrainSubsystem, 2.0),
            new RotateDegrees(30, driveTrainSubsystem, -2.0),
            new RotateDegrees(30, driveTrainSubsystem, 2.0),
            new RotateDegrees(30, driveTrainSubsystem, -2.0),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, 2.0),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new DriveForwardSeconds(0.15, -0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, 4.0),
            new DriveForwardSeconds(0.3,  0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, -4.0),
            new DriveForwardSeconds(0.3,  0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, 4.0),
            new DriveForwardSeconds(0.3,  0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem, -4.0),
            new DriveForwardSeconds(0.3,  0.8, driveTrainSubsystem)
        );

        addRequirements(driveTrainSubsystem, shootingSubsystem);       
    }
}
