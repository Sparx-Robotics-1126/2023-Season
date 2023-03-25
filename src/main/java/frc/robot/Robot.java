package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import frc.robot.Constants.AcquisitionConstants;
// import frc.robot.Constants.ChooserOptions;
import frc.robot.subsystem.PigeonSubsystem;

/**
 * The main controlling class of the Robot. Controls all subsystems via
 * specialized Controllers.
 */
public class Robot extends TimedRobot {

	private RobotContainer _robotContainer;

	// private String _autoSelected;
	// private final SendableChooser<String> _chooser = new SendableChooser<>();
	private Command m_autonomousCommand;

	/**
	 * 23 * This function is run when the robot is first started up and should be
	 * used for any
	 * 24 * initialization code.
	 * 25
	 */
	@Override
	public void robotInit() {

		_robotContainer = new RobotContainer();
	}

	/**
	 * 34 * This function is called every 20 ms, no matter the mode. Use this for
	 * items like diagnostics
	 * 35 * that you want ran during disabled, autonomous, teleoperated and test.
	 * 36 *
	 * 37 *
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and
	 * 38 * SmartDashboard integrated updating.
	 * 39
	 */
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
		_robotContainer.displayShuffleboard();
		
		// if(SmartDashboard.getBoolean("Testing", false)) {
		// _robotContainer.tuningPeriodic();
		// }
		// SmartDashboard.putBoolean("SWITCH", _robotContainer.getSwitch());
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

		m_autonomousCommand = _robotContainer.getAutonomousCommand();

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

		if (m_autonomousCommand != null)
			m_autonomousCommand.cancel();

		// _robotContainer.startTimer();
		_robotContainer.reset();
		_robotContainer.setToCoast();
	}

	@Override
	public void teleopPeriodic() {
		double ly = -_robotContainer.getOperatorController().getLeftY();
		double ry = -_robotContainer.getOperatorController().getRightY();

		_robotContainer.EndGameRumble();
		// _robotContainer.EndEndGameRumble();

		SmartDashboard.putNumber("LEFT_Y", ly);
		SmartDashboard.putNumber("RIGHT_Y", ry);
		SmartDashboard.putBoolean("Grabber State", _robotContainer.getGrabberState());
		
		_robotContainer.getAcquisition().setXPower(MathUtil.applyDeadband(ly,
		AcquisitionConstants.MANUAL_DEADBAND));
		_robotContainer.getAcquisition().setYPower(MathUtil.applyDeadband(ry,
		AcquisitionConstants.MANUAL_DEADBAND));
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() {
		System.out.println("********** ROBOT DISABLED ************");
		// _robotContainer.stopTimer();

		// For safety reasons, apply the drivetrain brakes when the robot
		// becomes disabled. Avoids coasting into people. Useful for
		// demonstrations when someone inexperienced is operating.

		if (Constants.DriveConstants.BRAKE_ON_DISABLE)
	        _robotContainer.applyBrakes();
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

	@Override
	public void disabledPeriodic() {
		// TODO Auto-generated method stub
		super.disabledPeriodic();
	}

}
