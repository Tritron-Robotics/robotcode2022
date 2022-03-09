package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;

public class JitterBugSequentialCommand extends SequentialCommandGroup {
    
    public JitterBugSequentialCommand(DriveTrainSubsystem driveTrainSubsystem, ShootingSubsystem shootingSubsystem) 
    {
        System.out.println("Autonomous command group");

        // drive forward and look for ball
        addCommands(
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
            new DriveForwardSeconds(0.15,  0.8, driveTrainSubsystem),
            new RotateDegrees(180, driveTrainSubsystem),
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

        addRequirements(driveTrainSubsystem, shootingSubsystem);       
    }
}
