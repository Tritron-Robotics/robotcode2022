package frc.robot.subsystems;
import static org.mockito.Mockito.*;

import static org.junit.Assert.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.*;
import frc.robot.Constants;

import org.junit.*;

public class DriveTrainSubsystemTest {

    @Test
    public void testInitialization() {
        // test if motors are running
        CANSparkMax rearLeft = mock(CANSparkMax.class); 
        CANSparkMax frontLeft = mock(CANSparkMax.class); ; 
        CANSparkMax rearRight = mock(CANSparkMax.class); ; 
        CANSparkMax frontRight = mock(CANSparkMax.class); ; 
        
        DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(rearLeft, frontLeft, rearRight, frontRight);

        // Assert
        assertEquals(0.0, rearLeft.get(), 10e-3);
        assertEquals(0.0, frontLeft.get(), 10e-3);
        assertEquals(0.0, rearRight.get(), 10e-3);
        assertEquals(0.0, frontRight.get(), 10e-3);
    }

    // @Test
    // public void testMotors() {
    //     // test if motors are running
    //     CANSparkMax rearLeft = mock(CANSparkMax.class); 
    //     CANSparkMax frontLeft = mock(CANSparkMax.class);
    //     CANSparkMax rearRight = mock(CANSparkMax.class); 
    //     CANSparkMax frontRight = mock(CANSparkMax.class); 
        
    //     DriveTrainSubsystem driveTrainSubsystem = new DriveTrainSubsystem(rearLeft, frontLeft, rearRight, frontRight);
    //     driveTrainSubsystem.tankDriveVolts(1.0, 1.0);

    //     // Assert
    //     assertTrue("Motors are rotating", rearLeft.getOutputCurrent() == 0);
    // }
}