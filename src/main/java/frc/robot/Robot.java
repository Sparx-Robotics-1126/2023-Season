package frc.robot;

import frc.controllers.Controller;
import frc.controllers.ControllerMappings;
import frc.controllers.TeleoperatedController;
import frc.controllers.AutonomousController;
import frc.controllers.Axis;
import frc.controllers.TestController;
import frc.drives.DrivesSensorInterface;
import frc.drives.DrivesSensors;
import frc.subsystem.Drives;
import frc.sensors.Limelight;
import edu.wpi.first.wpilibj.TimedRobot;



// import edu.wpi.first.wpilibj.Encoder;
// import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.motorcontrol.MotorController;
// import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The main controlling class of the Robot. Controls all subsystems via specialized Controllers.
 */
public class Robot extends TimedRobot
{
 //Sensors.
    private DrivesSensorInterface drivesSensors;
  private DifferentialDrive m_myRobot;
  private final XboxController m_driverController = new XboxController(0);

  private Axis driverLeftAxisY;
  private Axis driverLeftAxisX;
	private Axis driverRightAxis;
  private Axis driverRightTrigger;

  // private static final int leftDeviceID = 1; 
  // private static final int rightDeviceID = 2;
  private CANSparkMax m_leftMotorMaster;
  private CANSparkMax m_rightMotorMaster; 

    /**
     * The maximum amount of current in amps that should be permitted during motor operation.
     */
    private static final int MAX_CURRENT = 25;
 
    /**
     * The ideal voltage that the motors should attempt to match.
     */
    private static final double NOMINAL_VOLTAGE = 12;
  // private CANSparkMax m_leftMotorSlave;
  // private CANSparkMax m_rightMotorSlave; 

    //Possible controllers.
    // private TeleoperatedController teleopControls;
    // private AutonomousController autoControls;
    // private TestController testControls;
    
    //The robot subsystems.
    // private static Drives drives;

    //The acting Controller of the robot.
    // private Controller currentController;

   

    // private static Limelight limelight;

    //Keeps track of the current state of the robot.
    //private RobotState state;

    @Override
    public void robotInit()
    {
   

        
        //Initialize sensors.
        drivesSensors = new DrivesSensors();

        m_rightMotorMaster = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
        CANSparkMax rightMotorSlave = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);
        // m_rightMotorMaster.setInverted(true);
        configureMotor(m_rightMotorMaster, rightMotorSlave);

        m_leftMotorMaster = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
        CANSparkMax leftMotorSlave = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);
        configureMotor(m_leftMotorMaster, leftMotorSlave);

var leftEncoder = m_leftMotorMaster.getEncoder();
var rightEncoder =  m_rightMotorMaster.getEncoder();

drivesSensors.addEncoders(leftEncoder,rightEncoder);

m_myRobot = new DifferentialDrive(m_leftMotorMaster, m_rightMotorMaster);


//define as driver and operator
        // drivesSensors = driveSensors;

    
        // limelight = new Limelight();
        
        //Initialize Subsystems.
        // drives = new Drives(drivesSensors);
        
        // //Initialize Controllers.
        // teleopControls = new TeleoperatedController();
        // autoControls = new AutonomousController();
        // testControls = new TestController();

        //Start subsystem threads.
        // new Thread(drives).start();
    }
    private static void configureMotor(CANSparkMax master, CANSparkMax... slaves) 
    {
        master.restoreFactoryDefaults();
        master.set(0);
        master.setIdleMode(IdleMode.kCoast);
        master.enableVoltageCompensation(NOMINAL_VOLTAGE);
        master.setSmartCurrentLimit(MAX_CURRENT);
        master.setOpenLoopRampRate(1);

        for (CANSparkMax slave : slaves) 
        {
            slave.restoreFactoryDefaults();
            slave.follow(master);
            slave.setIdleMode(IdleMode.kCoast);
            slave.setSmartCurrentLimit(MAX_CURRENT);
            slave.setOpenLoopRampRate(1);
        }
    }
    @Override
    public void robotPeriodic() {
       // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.

    }

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

    System.out.println("********** AUTONOMOUS STARTED ************");
                    autoStarted();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //   case kCustomAuto:
    //     // Put custom auto code here
    //     break;
    //   case kDefaultAuto:
    //   default:
    //     // Put default auto code here
    //     break;
    // }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
        // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
   
    System.out.println("********** TELEOPERATED STARTED ************");
    // teleopStarted();
    // currentController = teleopControls;

    // m_driverJoyStick = new Joystick(0);
    //operatorJoystick = new Joystick(1);

    // //DRIVES
    // driverLeftAxisY = new Axis(m_driverJoyStick, ControllerMappings.XBOX_LEFT_Y, true);
    // driverLeftAxisX = new Axis(m_driverJoyStick, ControllerMappings.XBOX_LEFT_X, true);
    // driverRightAxis = new Axis(m_driverJoyStick, ControllerMappings.XBOX_RIGHT_Y, true);
    // driverRightTrigger = new Axis(m_driverJoyStick, ControllerMappings.XBOX_R2, true);


    // double leftAxisY = driverLeftAxisY.get();
    // double leftAxisX = driverLeftAxisX.get() * 0.5;

    // if (Math.abs(leftAxisY) >= 0.15 && leftAxisY < 0)
    //   leftAxisX = -leftAxisX;

    // // Robot.getDrives().setJoysticks(leftAxisY - leftAxisX, leftAxisY + leftAxisX);

    //  //Trigger Sensitivity Control
    //  if (driverRightTrigger.get() <= -0.8)
    //  {
    //    driverLeftAxisX.setSensitivity(0.4);
    //    driverLeftAxisY.setSensitivity(0.4);
    //    driverRightAxis.setSensitivity(0.4);
    //  }
    //  else
    //  {
    //    driverLeftAxisX.setSensitivity(1);
    //    driverLeftAxisY.setSensitivity(1);
    //    driverRightAxis.setSensitivity(1);
    //  }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_myRobot.tankDrive(m_driverController.getLeftY(), -m_driverController.getRightY());
    // m_myRobot.tankDrive(-m_driverJoyStick.getZ(), -m_driverJoyStick.getX());
  }

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
   // Cancels all running commands at the start of test mode.
  //  CommandScheduler.getInstance().cancelAll();
  
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
    	// currentController = autoControls;
        // state = RobotState.AUTO;
    }

    /**
     * Called when teleoperated begins.
     */
    private void teleopStarted()
    {
    	// drives.startDriverControlled();
    	// currentController = teleopControls;
    	// state = RobotState.TELE;
    }

    /**
     * Called when test begins.
     */
    private void testStarted()
    {
        // currentController = testControls;
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
        // return drives;
        return null;
    }



    /**
     * @return The Limelight currently in use by the robot.
     */
    // public static Limelight getLimelight()
    // {
    //     return limelight;
    // }
}
