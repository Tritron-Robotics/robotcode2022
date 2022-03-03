// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDriveCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  private boolean isFinished = false;
  Timer timer;
  RamseteController controller = new RamseteController(Constants.Kinematics.kRamseteB, Constants.Kinematics.kRamseteZeta);

  DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                    Constants.Kinematics.kvVoltSecondsPerMeter,
                                    Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
            Constants.Kinematics.kDriveKinematics,
          5);
            
   // Create config for trajectory
   TrajectoryConfig config =
   new TrajectoryConfig(Constants.Kinematics.kMaxSpeedMetersPerSecond,
                       Constants.Kinematics.kMaxAccelerationMetersPerSecondSquared)
       // Add kinematics to ensure max speed is actually obeyed
       .setKinematics(Constants.Kinematics.kDriveKinematics)
       // Apply the voltage constraint
       .addConstraint(autoVoltageConstraint);

  // An example trajectory to follow.  All units in meters.
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
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

  String trajectoryJSON = "paths/YourPath.wpilib.json";
  //TrajectoryConfig trajectoryJson =  

  //DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.Kinematics.kTrackwidthMeters);

  /** Creates a new AutoDrive. 
   * @param driveTrain The drive train subsystem.
   */
  public AutoDriveCommand(DriveTrainSubsystem driveTrain) {
    this.driveTrain = driveTrain;

    timer = new Timer();
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();

    // try {
    //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
    //   trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } 
    // catch (IOException ex) 
    // {
    //   DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    // }

    // Reset odometry to the starting pose of the trajectory.
    //driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    driveTrain.arcadeDrive(0, 0);

    //ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    Trajectory.State goal = trajectory.sample(timer.get()); // sample the trajectory at 3.4 seconds from the beginning
    Pose2d pose = driveTrain.getPose();
    
    ChassisSpeeds adjustedSpeeds = controller.calculate(pose, goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = Constants.Kinematics.kDriveKinematics.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;
    //System.out.println(controller.calculate(pose, goal));d
    
    left /= 13.0;
    right /= 13.0;

    System.out.println("Left: " + left + "  Right: " + right);
    driveTrain.tankDriveVolts(left, right);

    //   System.out.println(timer.get());

    //   driveTrain.arcadeDrive(Constants.Kinematics.arcadeDriveSpeedModifierVoltage, 0);
    // } 
    // else
    // {
    //   driveTrain.arcadeDrive(0, 0);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
