// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ShootingSubsystem;

/** Add your docs here. */
public class RobotContainer {
    // Drivetrain subsystem.
    private final DriveTrain robotDrive = new DriveTrain();
    private final ShootingSubsystem shootingSubsystem = new ShootingSubsystem();
    private final AutoDrive autoDrive; 
    private final Joystick controller = new Joystick(0);

    public RobotContainer() {
        Command driveCommand = new DriveCommand(
            robotDrive, 
            () -> -controller.getY(), 
            () -> -controller.getRawAxis(3));
        robotDrive.setDefaultCommand(driveCommand);

        Command shootCommand = new ShootCommand(
            shootingSubsystem,
            () -> controller.getRawButton(Constants.Controller.rightTrigger),
            () -> controller.getRawButton(Constants.Controller.leftTrigger));       
        shootingSubsystem.setDefaultCommand(shootCommand);

        autoDrive = new AutoDrive(robotDrive);
    }

    public Command getAutonomousCommand() {
        return autoDrive;
    }

    // Trajectory Generation
    public Command getOtherAuto() {
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                    Constants.Kinematics.kvVoltSecondsPerMeter,
                                    Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
            Constants.Kinematics.kDriveKinematics,
            10);

        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(Constants.Kinematics.kMaxSpeedMetersPerSecond,
                                Constants.Kinematics.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Kinematics.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            // Pass config
            config
        );

        RamseteCommand ramseteCommand = new RamseteCommand(
            exampleTrajectory,
            robotDrive::getPose,
            new RamseteController(Constants.Kinematics.kRamseteB, Constants.Kinematics.kRamseteZeta),
            new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                       Constants.Kinematics.kvVoltSecondsPerMeter,
                                       Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
            Constants.Kinematics.kDriveKinematics,
            robotDrive::getWheelSpeeds,
            new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
            new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            robotDrive::tankDriveVolts,
            robotDrive
        );       

        // Reset odometry to the starting pose of the trajectory.
        robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> robotDrive.tankDriveVolts(0, 0));
    }
}
