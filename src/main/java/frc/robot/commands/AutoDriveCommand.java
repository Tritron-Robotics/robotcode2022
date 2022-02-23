// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrainSubsystem;

public class AutoDriveCommand extends CommandBase {
  DriveTrainSubsystem driveTrain;
  private boolean isFinished = false;
  Timer timer;

  // Create a voltage constraint to ensure we don't accelerate too fast
  DifferentialDriveVoltageConstraint autoVoltageConstraint =
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
      driveTrain::getPose,
      new RamseteController(Constants.Kinematics.kRamseteB, Constants.Kinematics.kRamseteZeta),
      new SimpleMotorFeedforward(Constants.Kinematics.ksVolts,
                                 Constants.Kinematics.kvVoltSecondsPerMeter,
                                 Constants.Kinematics.kaVoltSecondsSquaredPerMeter),
      Constants.Kinematics.kDriveKinematics,
      driveTrain::getWheelSpeeds,
      new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
      new PIDController(Constants.Kinematics.kPDriveVel, 0, 0),
      // RamseteCommand passes volts to the callback
      driveTrain::tankDriveVolts,
      driveTrain
  );       
  

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
    // Reset odometry to the starting pose of the trajectory.
    driveTrain.resetOdometry(exampleTrajectory.getInitialPose());
    driveTrain.arcadeDrive(0, 0);

    ramseteCommand.schedule();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    
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
