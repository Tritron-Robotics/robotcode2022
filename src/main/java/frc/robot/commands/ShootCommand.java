package frc.robot.commands;

import java.lang.Character.Subset;
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
    private BooleanSupplier toggleFastShootInput; //James chose the name <3 (intake)
    private BooleanSupplier toggleSlowShootInput;
    //shouldve named it chrisIsTurnedOn;
    private boolean dontShootMeConstantly; //Toggle Shoot or Normal
    private boolean shootMeConstantly;

    boolean isIntaking = false;
    boolean isFastShooting = false;
    boolean isSlowShooting = false;

    /**
     * Constructor for the ShootCommand class
     * @param shootingSubsystem Subsystem for shooting motors
     * @param fastShootInput Boolean that determines whether we want to shoot or not
     * @param slowShootInput Boolean that determines if the intake motors are on or not
     */
    public ShootCommand(ShootingSubsystem shootingSubsystem, BooleanSupplier fastShootInput, BooleanSupplier slowShootInput, DoubleSupplier intakeInput, BooleanSupplier toggleFastShoot, BooleanSupplier toggleSlowShoot, BooleanSupplier reverseShootInput, boolean dontShootMeConstantly) {
        this.fastShootInput = fastShootInput;
        this.slowShootInput = slowShootInput;
        this.intakeInput = intakeInput;
        this.subsystem = shootingSubsystem;
        this.reverseShootInput = reverseShootInput;
        this.toggleFastShootInput = toggleFastShoot;
        this.toggleSlowShootInput = toggleSlowShoot;
        this.dontShootMeConstantly = dontShootMeConstantly;
        shootMeConstantly = !dontShootMeConstantly;
        addRequirements(shootingSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        subsystem.stopAllMotors();
        isIntaking = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        if(dontShootMeConstantly)
        {
            if (fastShootInput.getAsBoolean()) 
            {
                subsystem.fastShoot(1);
            } 
            else if (slowShootInput.getAsBoolean())
            {
                subsystem.slowShoot(1);
            } 
            else 
            {
                subsystem.stopShootingMotors();
            }
        }


        if (reverseShootInput.getAsBoolean())
        {
            subsystem.reverseShoot(1);
        }

        if(shootMeConstantly)
        {
            //System.out.println("isFastShooting: " + isFastShooting + "  isSlowShooting: " + isSlowShooting + "  toggle slow shoot: " + toggleSlowShootInput.getAsBoolean());

            if(toggleFastShootInput.getAsBoolean())
            {
                System.out.println("toggle fast shoot");
                isFastShooting = !isFastShooting;
                isSlowShooting = false;
            }
            
            if(isFastShooting)
            {
                System.out.println("fast shoot");

                subsystem.fastShoot(1);
            }
            else if (!isSlowShooting)
            {
                System.out.println("stop fast shoot");

                //System.out.println("Not fast shooting or toggle slow input");
                subsystem.fastShoot(0);             
            }

            if(toggleSlowShootInput.getAsBoolean())
            {
                System.out.println("toggle slow shoot");

                isSlowShooting = !isSlowShooting;
                isFastShooting = false;
            }
            if(isSlowShooting)
            {
                System.out.println("slow shoot");

                subsystem.slowShoot(1);
            }
            else if (!isFastShooting)
            {
                System.out.println("stop slow shoot");

                subsystem.slowShoot(0);
            }

            
        }
        //  if (chrisTurnOn.getAsBoolean())
        //  {
        //      isIntaking = !isIntaking;
        //      subsystem.spinIntake(1.0);
        //  } 
        // if(isIntaking)
        // {
        //     subsystem.spinIntake(1);
        // }
        // else
        // {
        //     subsystem.spinIntake(0);
        // }

        
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
