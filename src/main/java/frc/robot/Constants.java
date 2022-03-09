package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class Kinematics
    {
        public static final double tankDriveVoltage = 4.0;
        public static final double arcadeDriveVoltage = 0.65;

        //public static final double arcadeDrivePartyModeValue = 0.76;
        public static final double arcadeDrivePartyModeValue = 0.83;
        public static final double arcadeDrivePrecisionMode = 0.5;

        public static final double tankDrivePartyModeVoltage = 0.9;
        public static final double tankDrivePrecisionModeVoltage = 0.5; 

        public static final double fastShootVoltage = 9.25;

        public static final double slowShootVoltage = 5.75;
        public static final double reverseShootVoltage = 3;

        public static final double intakeVoltage = 5;


        // PLACEHOLDERS: Use FRC Characterization Tool for values.
        // voltage needed to barely get the robot to start moving
        public static final double ksVolts = 0.22; 
        
        //how much voltage is needed to hold the robot at a constant velocity.
        public static final double kvVoltSecondsPerMeter = 1.98; 

        //how much voltage is needed to accelerate the robot at a constant
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
        // Example value only - as above, this must be tuned for your drive.
        public static final double kPDriveVel = 10;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds.
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;

        // Track Robot Velocity, pass in horizontal distance between wheels.
        public static final double kTrackwidthMeters = Units.inchesToMeters(21.5);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(kTrackwidthMeters);
    }

    public static final class MotorConstants
    {
        public static final int rearLeftPort = 1;
        public static final int frontLeftPort = 3;
        public static final int rearRightPort = 2;
        public static final int frontRightPort = 4;
    }

    public static final class Controller
    {
        // port numbers for controller buttons
        public static final int rightTrigger = 8;
        public static final int leftTrigger = 7;
        
        public static final int rightBumper = 6;
        public static final int leftBumper = 5;
        public static final int xButton = 4;
        public static final int yButton = 1;

        
        // to get left stick y-axis: -controller.getY()
        // to get right stick y-axis:  -controller.getRawAxis(3)
        // to get any button that corresponds to a port number(like the right trigger): controller.getRawButton(portnumber)
    }
}
