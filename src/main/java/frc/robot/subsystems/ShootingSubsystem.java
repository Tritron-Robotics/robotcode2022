// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Subsystem for the motors that shoot the ball */
public class ShootingSubsystem extends SubsystemBase {

    //Motor controllers
    private CANSparkMax intakeMotor;
    private CANSparkMax topLeftShootMotor;
    private CANSparkMax topRightShootMotor;

    // Initialize the motors
    public ShootingSubsystem() {
        intakeMotor = new CANSparkMax(5, MotorType.kBrushless);
        topLeftShootMotor = new CANSparkMax(8, MotorType.kBrushed);
        topRightShootMotor = new CANSparkMax(7, MotorType.kBrushed);
    }

    /**
     * Shoots the ball
     * @param input Controller input
     */
    public void shoot(double input)
    {
        topLeftShootMotor.setVoltage(-input * 8);
        topRightShootMotor.setVoltage(input * 8);
        //spinIntake(input);
    }

    /**
     * Activates the intake system
     * @param input Controller input
     */
    public void spinIntake(double input)
    {
        intakeMotor.setVoltage(input * 10); 
    }

    public void stopMotors()
    {
        topLeftShootMotor.setVoltage(0);
        topRightShootMotor.setVoltage(0);
        intakeMotor.setVoltage(0);
    }
}
