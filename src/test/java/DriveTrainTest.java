import static org.junit.Assert.*;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.simulation.DoubleSolenoidSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
//import frc.robot.Constants.IntakeConstants;
import org.junit.*;

public class DriveTrainTest {
  public static final double DELTA = 1e-2; // acceptable deviation range
  //Intake intake;
  PWMSim simMotor;
  //DoubleSolenoidSim simPiston;

  @Before // this method will run before each test
  public void setup() {
    assert HAL.initialize(500, 0); // initialize the HAL, crash if failed
    //intake = new Intake(); // create our intake
    //simMotor = new PWMSim(IntakeConstants.MOTOR_PORT); // create our simulation PWM motor controller
    //simPiston = new DoubleSolenoidSim(PneumaticsModuleType.CTREPCM, IntakeConstants.PISTON_FWD, IntakeConstants.PISTON_REV); // create our simulation solenoid
  }

  @After // this method will run after each test
  public void shutdown() throws Exception 
  {
    //intake.close(); // destroy our intake object
  }

  @Test // marks this method as a test
  public void doesntWorkWhenClosed() {
    //intake.retract(); // close the intake
    //intake.activate(0.5); // try to activate the motor
    //assertEquals(0.0, simMotor.getSpeed(), DELTA); // make sure that the value set to the motor is 0
  }

  @Test
  public void worksWhenOpen() {
    //intake.deploy();
    //intake.activate(0.5);
    //assertEquals(0.5, simMotor.getSpeed(), DELTA);
  }

  @Test
  public void retractTest() {
    //intake.retract();
    //assertEquals(DoubleSolenoid.Value.kReverse, simPiston.get());
  }

  @Test
  public void deployTest() {
    //intake.deploy();
    //assertEquals(DoubleSolenoid.Value.kForward, simPiston.get());
  }
}