// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.autocommands.AutoAlign;
import frc.robot.commands.autocommands.AutonomousCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ShootingSubsystem;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {
    // Drivetrain subsystem.
    private DriveTrainSubsystem robotDriveSubsystem; 
    private ShootingSubsystem shootingSubsystem;

    private final Joystick controller = new Joystick(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    AutonomousCommand autoCommand;

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
        robotDriveSubsystem = new DriveTrainSubsystem(rearLeft, frontLeft, rearRight, frontRight);

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
        Command arcadeDriveCommand = new ArcadeDriveCommand(
            robotDriveSubsystem, 
            () -> -controller.getY(), 
            () -> controller.getX(), 
            () -> controller.getRawButton(Constants.Controller.leftBumper),
            () -> controller.getRawButton(Constants.Controller.yButton));

        //autoAlignCommand = new AutoAlign(robotDriveSubsystem);

        //bButton.whenPressed(autoAlignCommand);

        Command shootCommand = new ShootCommand(
            shootingSubsystem,
            () -> controller.getRawButton(Constants.Controller.leftTrigger),
            () -> controller.getRawButton(Constants.Controller.rightTrigger),          
            () -> controller.getRawAxis(3),
            () -> controller.getRawButton(Constants.Controller.xButton));       
        
        shootingSubsystem.setDefaultCommand(shootCommand);

        autoCommand = new AutonomousCommand(robotDriveSubsystem, shootingSubsystem);
        
        robotDriveSubsystem.setDefaultCommand(arcadeDriveCommand);

        // SmartDashboard.putData("Rotate 180", new RotateDegrees(180, robotDriveSubsystem, 2.0));
        // SmartDashboard.putData("Drive Forward 2 feet", new DriveForwardDistance(robotDriveSubsystem, 2.0, 1.0));
        // SmartDashboard.putData("Track", new TrackObjectCommand(robotDriveSubsystem));
        

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
        return autoChooser.getSelected();
    }
}
