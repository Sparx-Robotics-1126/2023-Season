package frc.robot;


import frc.subsystem.Drives;
import frc.sensors.Limelight;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The main controlling class of the Robot. Controls all subsystems via specialized Controllers.
 */
public class Robot extends TimedRobot
{
 
 
  private final Timer m_timer = new Timer();
  private Command m_autonomousCommand;
  
  private RobotContainer m_robotContainer;
  // private Command m_autonomousCommand;

    @Override
    public void robotInit()
    {
      m_robotContainer = new RobotContainer();
      

        // limelight = new Limelight();
      
        // //Initialize Controllers.
        // teleopControls = new TeleoperatedController();
        // autoControls = new AutonomousController();
        // testControls = new TestController();

        //Start subsystem threads.
        // new Thread(drives).start();
    }
  
    @Override
    public void robotPeriodic() {
       // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
CommandScheduler.getInstance().run();
var pitch = m_robotContainer.getPitch();
SmartDashboard.putNumber("PITCH", pitch);

// SmartDashboard.putNumber("Position X", m_robotContainer.getDrives().getPose().getX());
// SmartDashboard.putNumber("Position Y", m_robotContainer.getDrives().getPose().getY());

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
  // Drive for 2 seconds
  // if (m_timer.get() < 2.0) {
  //   System.out.println("Moving forward");
  //   // Drive forwards half speed, make sure to turn input squaring off
  //   m_myRobot.tankDrive(-0.1, 0.1, false);
  // } else {
  //   System.out.println("Moving stopped");;
  //   m_myRobot.stopMotor(); // stop robot
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

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    
    // teleopStarted();
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
   // Cancels all running commands at the start of test mode.
   CommandScheduler.getInstance().cancelAll();
  
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
        // m_timer.reset();
        // m_timer.start();

      //  m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        /*
         * String autoSelected = SmartDashboard.getString("Auto Selector",
         * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
         * = new MyAutoCommand(); break; case "Default Auto": default:
         * autonomousCommand = new ExampleCommand(); break; }
         */
    
        // schedule the autonomous command (example)
        if (m_autonomousCommand != null) {
          m_autonomousCommand.schedule();
        }
    }

    /**
     * Called when teleoperated begins.
     */
    private void teleopStarted()
    {

      // double sensitivity =.5;

      // if (m_driverController.getRightBumperPressed()){
        
      //   sensitivity = 0.25;
      //   System.out.println("sensitivity set: " + sensitivity);
      // }126

      // if (m_driverController.getRightBumperReleased()){
      //   sensitivity = 1.0;
      //   System.out.println("sensitivity set: " + sensitivity);
      // }
      // System.out.println("sensitivity set: "+ m_driverController.getLeftY() * sensitivity);
      
      // m_myRobot.tankDrive(m_driverController.getLeftY() * sensitivity, -m_driverController.getRightY()*sensitivity);
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
     * @return The instance of the Drives subsystem currently in use by the robot. Null if the robot has not been initialized yet.
     */
    // public static Drives getDrives()
    // {
    //     // return drives;
    //     return null;
    // }



    /**
     * @return The Limelight currently in use by the robot.
     */
    // public static Limelight getLimelight()
    // {
    //     return limelight;
    // }
}
