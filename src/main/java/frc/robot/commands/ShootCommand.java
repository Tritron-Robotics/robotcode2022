package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShootingSubsystem;

public class ShootCommand extends CommandBase {

    public ShootingSubsystem subsystem;
    public BooleanSupplier fastShootInput;
    public BooleanSupplier slowShootInput;
    public BooleanSupplier reverseShootInput;
    private DoubleSupplier intakeInput;

    /**
     * Constructor for the ShootCommand class
     * @param shootingSubsystem Subsystem for shooting motors
     * @param fastShootInput Boolean that determines whether we want to shoot or not
     * @param slowShootInput Boolean that determines if the intake motors are on or not
     */
    public ShootCommand(ShootingSubsystem shootingSubsystem, BooleanSupplier fastShootInput, BooleanSupplier slowShootInput, DoubleSupplier intakeInput, BooleanSupplier reverseShootInput) {
        this.fastShootInput = fastShootInput;
        this.slowShootInput = slowShootInput;
        this.intakeInput = intakeInput;
        this.subsystem = shootingSubsystem;
        this.reverseShootInput = reverseShootInput;
        addRequirements(shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.stopAllMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if (fastShootInput.getAsBoolean()) 
        {
            subsystem.fastShoot(1);
        } 
        else if (slowShootInput.getAsBoolean())
        {
            subsystem.slowShoot(1);
        } 
        else if (reverseShootInput.getAsBoolean())
        {
            subsystem.reverseShoot(1);
        }
        else 
        {
            System.out.print("Not shoot input so stopping motors");
            subsystem.stopShootingMotors();
        }

        subsystem.spinIntake(-intakeInput.getAsDouble());
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
