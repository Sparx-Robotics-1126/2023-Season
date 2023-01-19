package frc.robot;

import frc.controllers.Controller;
import frc.controllers.TeleoperatedController;
import frc.controllers.AutonomousController;
import frc.controllers.TestController;

import frc.subsystem.Drives;
import frc.drives.DrivesSensorInterface;
import frc.drives.DrivesSensors;

import frc.sensors.Limelight;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The main controlling class of the Robot. Controls all subsystems via specialized Controllers.
 */
public class Robot extends TimedRobot
{
//new wpilib timed robot
    private static final String kDefaultAuto = "Default";
    private static final String kCustomAuto = "My Auto";
    private String m_autoSelected;
    private final SendableChooser<String> m_chooser = new SendableChooser<>();
    
    
    //Every possible state of control for the robot.
    // public enum RobotState
    // {
	// 	STANDBY,
	// 	AUTO,
    //     TELE,
    //     TEST;
	// }

    //Possible controllers.
    private TeleoperatedController teleopControls;
    private AutonomousController autoControls;
    private TestController testControls;
    
    //The robot subsystems.
    private static Drives drives;

    //The acting Controller of the robot.
    private Controller currentController;

    //Sensors.
    private DrivesSensorInterface drivesSensors;

    private static Limelight limelight;

    //Keeps track of the current state of the robot.
    //private RobotState state;

    @Override
    public void robotInit()
    {
        //from new
        m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
        m_chooser.addOption("My Auto", kCustomAuto);
        SmartDashboard.putData("Auto choices", m_chooser);


//        state = RobotState.STANDBY; //When robot turns on, we don't want anything running in the background.
        
        //Initialize sensors.
        drivesSensors = new DrivesSensors();
    
        limelight = new Limelight();
        
        //Initialize Subsystems.
        drives = new Drives(drivesSensors);
        
        //Initialize Controllers.
        teleopControls = new TeleoperatedController();
        autoControls = new AutonomousController();
        testControls = new TestController();

        //Start subsystem threads.
        new Thread(drives).start();
    }

    @Override
    public void robotPeriodic() {}

 /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    System.out.println("********** AUTONOMOUS STARTED ************");
                    autoStarted();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    System.out.println("********** TELEOPERATED STARTED ************");
    teleopStarted();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("********** ROBOT DISABLED ************");
    disabledStarted();        
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
                        System.out.println("********** TEST STARTED ************");
    testStarted();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

    private void disabledStarted()
    {
        // state = RobotState.STANDBY;
    }

    /**
     * Called when autonomous begins.
     */
    private void autoStarted()
    {
    	currentController = autoControls;
        // state = RobotState.AUTO;
    }

    /**
     * Called when teleoperated begins.
     */
    private void teleopStarted()
    {
    	drives.startDriverControlled();
    	currentController = teleopControls;
    	// state = RobotState.TELE;
    }

    /**
     * Called when test begins.
     */
    private void testStarted()
    {
        currentController = testControls;
        // state = RobotState.TEST;
    }

    /**
     * The main loop of the robot. Ran every update/tick of the RIO.
     */
    // private void mainLoop() 
    // {
    //     switch (state)
    //     {
    //         case STANDBY:
    //             return;
    //         case AUTO:
    //             //This can be used to give grive (drive?) control after done
    //         	//Also used if semi-auto things are happening
    //         	//NOTICE THERE IS NO BREAK HERE
    //         case TELE:
    //         case TEST:
    //             currentController.execute(); //Calls the current controller (auto/teleop/test)
    //     }
    // }

//     @Override
//     public void startCompetition() 
//     {
//         System.out.println("******** ROBOT INIT ********");
       
//         DriverStationJNI.observeUserProgramStarting();
//         robotInit();

//         System.out.println("************ ENGAGING MAIN LOOP ************");
// /*System.out.println("Robot disabled " + isDisabled() + " State: " + state);*/
//         while (true)
//         {
//             if (!isDisabled())
//             {
//                 if (isAutonomous() && state != RobotState.AUTO)
//                 {
//                     System.out.println("********** AUTONOMOUS STARTED ************");
//                     autoStarted();
//                     DriverStationJNI.observeUserProgramAutonomous();
//                 }
//                 else if (isTeleop() && state != RobotState.TELE)
//                 {
//                     teleopStarted();
//                     System.out.println("********** TELEOPERATED STARTED ************");
//                     DriverStationJNI.observeUserProgramTeleop();
//                 }
//                 else if (isTest() && state != RobotState.TEST)
//                 {
//                     System.out.println("********** TEST STARTED ************");
//                     testStarted();
//                     DriverStationJNI.observeUserProgramTest();
//                 }
//             }
//             else if (state != RobotState.STANDBY)
//             {
//                 System.out.println("********** ROBOT DISABLED ************");
//                 disabledStarted();        
//                 DriverStationJNI.observeUserProgramDisabled();
//             }

//             SmartDashboard.updateValues();
//             mainLoop();
//         }
    // }

    // @Override
    // public void endCompetition() 
    // {
    
    // }
    
    /**
     * @return The instance of the Drives subsystem currently in use by the robot. Null if the robot has not been initialized yet.
     */
    public static Drives getDrives()
    {
        return drives;
    }



    /**
     * @return The Limelight currently in use by the robot.
     */
    public static Limelight getLimelight()
    {
        return limelight;
    }
}
