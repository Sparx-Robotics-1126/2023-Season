package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ChooserOptions;


/**
 * The main controlling class of the Robot. Controls all subsystems via
 * specialized Controllers.
 */
public class Robot extends TimedRobot {

  // Options for the Sendable Chooser


  private String _autoSelected;
  private final SendableChooser<String> _chooser = new SendableChooser<>();
  // private final Timer m_timer = new Timer();
  private Command m_autonomousCommand;

  private RobotContainer _robotContainer;

  @Override
  public void robotInit() {

    _robotContainer = new RobotContainer();
    _chooser.setDefaultOption("Short", ChooserOptions.kAutoShort);
    _chooser.addOption("Long", ChooserOptions.kAutoLong);
    _chooser.addOption("Measure", ChooserOptions.kDriveMeasure);
    SmartDashboard.putData("AUTO CHOICES", _chooser);

    
  }

  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
SmartDashboard.putBoolean("SWITCH", _robotContainer.getSwitch());
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different
   * autonomous modes using the dashboard. The sendable chooser code works with
   * the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the
   * chooser code and
   * uncomment the getString line to get the auto name from the text box below the
   * Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure
   * below with additional strings. If using the SendableChooser make sure to add
   * them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    System.out.println("********** AUTONOMOUS STARTED ************");

    _robotContainer.reset();

    _autoSelected = _chooser.getSelected();
    //  System.out.println("Auto Selected: " + m_autoSelected);
    switch (_autoSelected) {

      case ChooserOptions.kAutoShort:
        m_autonomousCommand = _robotContainer.getShortAutoCommand();
        break;

      case ChooserOptions.kAutoLong:
        m_autonomousCommand = _robotContainer.getLongAutoCommand();
        break;

      case ChooserOptions.kDriveMeasure:
        m_autonomousCommand = _robotContainer.getDriveMeasurements();
        break;
    }


    if (m_autonomousCommand != null) {
      _robotContainer.setToCoast();
      m_autonomousCommand.schedule();
    }
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

    _robotContainer.reset();
    _robotContainer.setToCoast();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    System.out.println("********** ROBOT DISABLED ************");
    // disabledStarted();
  }


  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    System.out.println("********** TEST STARTED ************");
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();

  }



}
