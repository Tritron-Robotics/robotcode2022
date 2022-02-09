// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrainSubsystem;

public class BullCommand extends CommandBase {

	DriveTrainSubsystem driveTrain;
	private boolean isFinished = false;
	private boolean isLockedIn = false;
	Timer timer;

	DoubleSupplier testInput;

	NetworkTable limelightNetworkTable;
	NetworkTableEntry tx;
	NetworkTableEntry ty;
	NetworkTableEntry ta;
	NetworkTableEntry tv;

	private static final int buffer = 4;

	double x, y, area;
	boolean isTrackingObject;

	/**
	 * Creates a new AutoDrive.
	 * 
	 * @param driveTrain The drive train subsystem.
	 */
	public BullCommand(DriveTrainSubsystem driveTrain) {
		this.driveTrain = driveTrain;
		limelightNetworkTable = NetworkTableInstance.getDefault().getTable("limelight");
		tx = limelightNetworkTable.getEntry("tx");
		ty = limelightNetworkTable.getEntry("ty");
		tv = limelightNetworkTable.getEntry("tv");
		ta = limelightNetworkTable.getEntry("ta");

		timer = new Timer();

		addRequirements(driveTrain);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer.reset();
		timer.start();

		driveTrain.arcadeDrive(0, 0);
		SmartDashboard.putNumber("ArcadeDriveY", 0);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		getLimelightData();

		if (isTrackingObject) {
			if (timer.get() == 0.0) {
				timer.start();
				isLockedIn = true;
			}
		} else if (!isLockedIn) {
			timer.reset();
			findObjectToTrack();
		}

		// if is tracking object for more than 0.5 seconds
		if (timer.get() > 0.0) {
			driveTrain.arcadeDrive(0, 0);
			if (timer.get() > 0.5 && timer.get() <= 0.85) {
				driveBackward(0.3);
			} else if (timer.get() > 0.85 && timer.get() <= 1.15) {
				driveForward(0.3);
			} else if (timer.get() > 1.15 && timer.get() <= 1.5) {
				driveBackward(0.3);
			} else if (timer.get() > 1.5 && timer.get() <= 2.5) {
				driveForward(0.5);
			} else if (timer.get() >= 2.5) {
				isLockedIn = false;
			} else {
				System.out.println("Resting");
			}
		} else {
			rotateToTrackedObject();
		}
	}

	void getLimelightData() {
		// read values periodically
		x = tx.getDouble(0.0);
		y = ty.getDouble(0.0);
		area = ta.getDouble(0.0);
		isTrackingObject = tv.getDouble(0.0) == 1.0 ? true : false;

		// post to smart dashboard periodically
		SmartDashboard.putNumber("LimelightX", x);
		SmartDashboard.putNumber("LimelightY", y);
		SmartDashboard.putNumber("LimelightArea", area);
		SmartDashboard.putBoolean("IsTrackingObject", isTrackingObject);
	}

	void rotateToTrackedObject() {
		if (x > buffer) {
			driveTrain.arcadeDrive(0, 0.3);
		} else if (x < -buffer) {
			driveTrain.arcadeDrive(0, -0.3);
		}
	}

	void findObjectToTrack() {
		driveTrain.arcadeDrive(0, 0.3);
	}

	void driveForward(double speed) {
		driveTrain.arcadeDrive(speed, 0.0);
	}

	void driveBackward(double speed) {
		driveTrain.arcadeDrive(-speed, 0.0);
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
