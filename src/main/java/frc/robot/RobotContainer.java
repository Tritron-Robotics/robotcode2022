// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.Random;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.BullMode;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autocommands.AutoAlign;
import frc.robot.commands.autocommands.AutonomousCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {
    // Drivetrain subsystem.
    private DriveTrainSubsystem driveTrainSubsystem; 
    private ShootingSubsystem shootingSubsystem;

    private final Joystick controller = new Joystick(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    private AutoAlign autoAlignCommand;

    AutonomousCommand autoCommand;
    BullMode bullMode;

    //private AutoAlign autoAlignCommand;

    JoystickButton bButton = new JoystickButton(controller, Constants.Controller.bButton);
    /**
     * Initializes all robot commands.
     */
    public RobotContainer() {

        InitializeSubsystems();
        AssignCommands();
    }

    
    /**
     * Initializes the subsystems. 
     */
    private void InitializeSubsystems()
    {
        CANSparkMax rearLeft = new CANSparkMax(Constants.MotorConstants.rearLeftPort, MotorType.kBrushless); 
        CANSparkMax frontLeft = new CANSparkMax(Constants.MotorConstants.frontLeftPort, MotorType.kBrushless); 
        CANSparkMax rearRight = new CANSparkMax(Constants.MotorConstants.rearRightPort, MotorType.kBrushless); 
        CANSparkMax frontRight = new CANSparkMax(Constants.MotorConstants.frontRightPort, MotorType.kBrushless); 
        driveTrainSubsystem = new DriveTrainSubsystem(rearLeft, frontLeft, rearRight, frontRight);

        CANSparkMax intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        CANSparkMax topLeftShootMotor = new CANSparkMax(8, MotorType.kBrushed);
        CANSparkMax topRightShootMotor = new CANSparkMax(7, MotorType.kBrushed);
        shootingSubsystem = new ShootingSubsystem(intakeMotor, topLeftShootMotor, topRightShootMotor);       
    }

    
    /**
     * Assigns the commands for driving and shooting.
     */
    private void AssignCommands()
    {
        Command defaultDriveCommand = new ArcadeDriveCommand(
            driveTrainSubsystem, 
            () -> -controller.getY(), 
            () -> controller.getX(), 
            () -> controller.getRawButton(Constants.Controller.leftBumper),
            () -> controller.getRawButton(Constants.Controller.yButton));

        Command defaultShootCommand = new ShootCommand(
            shootingSubsystem,
            () -> controller.getRawButton(Constants.Controller.leftTrigger),
            () -> controller.getRawButton(Constants.Controller.rightTrigger),          
            () -> controller.getRawAxis(3),
            () -> controller.getRawButtonPressed(Constants.Controller.leftTrigger),
            () -> controller.getRawButtonPressed(Constants.Controller.rightTrigger),
            () -> controller.getRawButton(Constants.Controller.xButton),
            false); 

        //JESUS (CHRIST)
        Command jesusDriveCommand = new ArcadeDriveCommand(
            driveTrainSubsystem, 
            () -> -controller.getY(), 
            () -> controller.getRawAxis(2), 
            () -> controller.getRawButton(Constants.Controller.leftBumper),
            () -> controller.getRawButton(Constants.Controller.yButton));

        Command jesusShootCommand = new ShootCommand(
            shootingSubsystem,
            () -> controller.getRawButton(Constants.Controller.leftTrigger),
            () -> controller.getRawButton(Constants.Controller.rightTrigger),          
            () -> 
            {
                if(controller.getRawButton(Constants.Controller.rightBumper))
                {
                    return -1;
                }
                else if(controller.getRawButton(Constants.Controller.xButton))
                {
                    return 1;
                }
                else
                {
                    return 0;
                }
            },
            () -> controller.getRawButtonPressed(Constants.Controller.leftTrigger),
            () -> controller.getRawButtonPressed(Constants.Controller.rightTrigger),
            () -> controller.getRawButton(Constants.Controller.xButton),
            false); 

        autoAlignCommand = new AutoAlign(driveTrainSubsystem);

        bButton.whenPressed(autoAlignCommand);

        
        shootingSubsystem.setDefaultCommand(defaultShootCommand);

        autoCommand = new AutonomousCommand(driveTrainSubsystem, shootingSubsystem);
        //bullMode = new BullMode(driveTrainSubsystem, shootingSubsystem);
        driveTrainSubsystem.setDefaultCommand(defaultDriveCommand);

        // SmartDashboard.putData("Rotate 180", new RotateDegrees(180, robotDriveSubsystem, 2.0));
        // SmartDashboard.putData("Drive Forward 2 feet", new DriveForwardDistance(robotDriveSubsystem, 2.0, 1.0));
        // SmartDashboard.putData("Track", new TrackObjectCommand(robotDriveSubsystem));
        
        //controlChooser.addOption("Chris", new Object());
        autoChooser.setDefaultOption("Default Auto", autoCommand);
        SmartDashboard.putData(autoChooser);
    }
    /*
    if(karel=facingWest){
        move();
        putBall();
    }
    toggle pause
    */
    /**
     * Gets the autonomous command
     * @return the currently selected autonomous command.
     */
    public Command getAutonomousCommand() 
    {  
        //return getPathCommand();
        return autoChooser.getSelected();
    }

    public Command getPathCommand() {

        System.out.println("Get path command");
        // Create a voltage constraint to ensure we don't accelerate too fast
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    Constants.Kinematics.ksVolts,
                    Constants.Kinematics.kvVoltSecondsPerMeter,
                    Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
                    Constants.Kinematics.kDriveKinematics,
                10);
    
        // Create config for trajectory
        TrajectoryConfig config =
            new TrajectoryConfig(
                Constants.Kinematics.kMaxSpeedMetersPerSecond,
                Constants.Kinematics.kMaxAccelerationMetersPerSecondSquared)
                // Add kinematics to ensure max speed is actually obeyed
                .setKinematics(Constants.Kinematics.kDriveKinematics)
                // Apply the voltage constraint
                .addConstraint(autoVoltageConstraint);
    
        // An example trajectory to follow.  All units in meters.
        edu.wpi.first.math.trajectory.Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                config);
    
        RamseteCommand ramseteCommand =
            new RamseteCommand(
                exampleTrajectory,
                driveTrainSubsystem::getPose,
                new RamseteController(Constants.Kinematics.kRamseteB, Constants.Kinematics.kRamseteZeta),
                new SimpleMotorFeedforward(
                    Constants.Kinematics.ksVolts,
                    Constants.Kinematics.kvVoltSecondsPerMeter,
                    Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
                    Constants.Kinematics.kDriveKinematics,
                driveTrainSubsystem::getWheelSpeeds,
                new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
                new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                driveTrainSubsystem::tankDriveVolts,
                driveTrainSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        driveTrainSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return ramseteCommand.andThen(() -> System.out.println("after command"));
    return ramseteCommand.andThen(() -> driveTrainSubsystem.tankDriveVolts(0, 0));
    }
}
