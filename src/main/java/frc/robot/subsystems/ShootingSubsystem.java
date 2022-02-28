// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for the motors that shoot the ball */
public class ShootingSubsystem extends SubsystemBase {

    //Motor controllers
    private CANSparkMax intakeMotor;
    private CANSparkMax topLeftShootMotor;
    private CANSparkMax topRightShootMotor;

    // Initialize the motors
    public ShootingSubsystem(CANSparkMax intakeMotor, CANSparkMax topLeftShootMotor, CANSparkMax topRightShootMotor) {
        this.intakeMotor = intakeMotor;
        this.topLeftShootMotor = topLeftShootMotor;
        this.topRightShootMotor = topRightShootMotor;

        topRightShootMotor.setInverted(true);
    }

    /**
     * Shoots the ball fast to make it into the top hub
     * @param input Controller input
     */
    public void fastShoot(double input)
    {
        topLeftShootMotor.setVoltage(-input * Constants.Kinematics.fastShootVoltage);
        topRightShootMotor.setVoltage(-input * Constants.Kinematics.fastShootVoltage);
    }

    /**
     * Shoots the ball slowly to make it into the low hub
     * @param input Controller input
     */
    public void slowShoot(double input)
    {
        topLeftShootMotor.setVoltage(-input * Constants.Kinematics.slowShootVoltage);
        topRightShootMotor.setVoltage(-input * Constants.Kinematics.slowShootVoltage);
    }

    /**
     * Shoots the ball slowly to make it into the low hub
     * @param input Controller input
     */
    public void reverseShoot(double input)
    {
        topLeftShootMotor.setVoltage(input * Constants.Kinematics.reverseShootVoltage);
        topRightShootMotor.setVoltage(input * Constants.Kinematics.reverseShootVoltage);
    }

    /**
     * Activates the intake system
     * @param input Controller input
     */
    public void spinIntake(double input)
    {
        intakeMotor.setVoltage(input * Constants.Kinematics.intakeVoltage); 
    }

    public void stopShootingMotors()
    {
        topLeftShootMotor.setVoltage(0);
        topRightShootMotor.setVoltage(0);
    }

    public void stopMotors()
    {
        topLeftShootMotor.setVoltage(0);
        topRightShootMotor.setVoltage(0);
        intakeMotor.setVoltage(0);
    }
}
