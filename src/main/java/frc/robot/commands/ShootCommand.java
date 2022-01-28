package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootCommand extends CommandBase {

    public ShootingSubsystem subsystem;
    public BooleanSupplier shootInput;
    public BooleanSupplier intakeInput;

    /**
     * Constructor for the ShootCommand class
     * @param shootingSubsystem Subsystem for shooting motors
     * @param shootInput Boolean that determines whether we want to shoot or not
     * @param intakeInput Boolean that determines if the intake motors are on or not
     */
    public ShootCommand(ShootingSubsystem shootingSubsystem, BooleanSupplier shootInput, BooleanSupplier intakeInput) {
        this.shootInput = shootInput;
        this.intakeInput = intakeInput;
        this.subsystem = shootingSubsystem;
        addRequirements(shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.stopMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        //System.out.println("Shoot input: " + shootInput.getAsBoolean());
       
        if (shootInput.getAsBoolean()) {
            subsystem.shoot(-1);
        } else {
            subsystem.shoot(0);
        }

        if(intakeInput.getAsBoolean())
        {
            subsystem.spinIntake(1);
        }
        else
        {
            subsystem.spinIntake(0);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) 
    {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
