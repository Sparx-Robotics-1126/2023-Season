package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.Constants.ChooserOptions;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.BalanceShortRobot;
import frc.robot.commands.BalanceLongRobot;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.commands.DriveMeasurements;
import frc.robot.commands.MoveTo;
import frc.robot.commands.ScoreCommunity;
import frc.robot.commands.TurnPID;
import frc.robot.commands.TurnRight;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PigeonSubsystem;
import frc.robot.commands.ReturnToHome;
import frc.robot.sensors.Limelight;
import static frc.robot.Constants.FieldConstants.*;

/**
 * The main controlling class of the Robot. Controls all subsystems via
 * specialized Controllers.
 */
public class Robot extends TimedRobot 
{
	private final PigeonSubsystem m_pigeon;
	private final DriveSubsystem m_robotDrive;
	private final AcquisitionSubsystem m_robotAcquisition;
	private final XboxController m_driverController;
	private final XboxController m_operatorController;
	private final EventLoop m_controllerEventLoop;
	private final Timer m_Timer;

	private PIDController m_pid;
	private String _autoSelected;
	private final SendableChooser<String> _chooser = new SendableChooser<>();
	// private final Timer m_timer = new Timer();
	private Command m_autonomousCommand;

	public Robot()
	{
		m_Timer = new Timer();
		Limelight limeLight = new Limelight();
		limeLight.enableVision();

		m_driverController = new XboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT);
		m_operatorController = new XboxController(Constants.XBOX_OPERATOR_CONTROLLER_PORT);
		m_pigeon = new PigeonSubsystem();
		m_controllerEventLoop = new EventLoop();

		m_robotAcquisition = new AcquisitionSubsystem();

		m_robotDrive = new DriveSubsystem(m_pigeon, m_Timer);
		m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED);
		m_robotDrive.setDefaultCommand(
			new RunCommand(
				() -> m_robotDrive.tankDrive(
					(m_driverController.getLeftY()), 
					m_driverController.getRightY()),
			m_robotDrive));

		m_pid = new PIDController(0, 0, 0);

		configureDriverButtonBindings();
		configureOperatorButtons();
	}

	@Override
	public void robotInit() 
	{
		m_pid = getTurnPID();
		_chooser.setDefaultOption("Short", ChooserOptions.kAutoShort);
		_chooser.addOption("Long", ChooserOptions.kAutoLong);
		_chooser.addOption("Measure", ChooserOptions.kDriveMeasure);
		_chooser.addOption("Score and Leave Community",ChooserOptions.kScoreCommunity);
		SmartDashboard.putData("AUTO CHOICES", _chooser);
	}

	@Override
	public void robotPeriodic() 
	{
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled
		// commands, running already-scheduled commands, removing finished or
		// interrupted commands,
		// and running subsystem periodic() methods. This must be called from the
		// robot's periodic
		// block in order for anything in the Command-based framework to work.

		CommandScheduler.getInstance().run();
		m_controllerEventLoop.poll();
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
	public void autonomousInit() 
	{
		System.out.println("********** AUTONOMOUS STARTED ************");

		reset();

		_autoSelected = _chooser.getSelected();

		switch (_autoSelected) 
		{
			case ChooserOptions.kAutoShort:
				m_autonomousCommand = getShortAutoCommand();
				break;

			case ChooserOptions.kAutoLong:
				m_autonomousCommand = getLongAutoCommand();
				break;

			case ChooserOptions.kDriveMeasure:
				m_autonomousCommand = getDriveMeasurements();
				break;

			case ChooserOptions.kScoreCommunity:
				m_autonomousCommand = getScoreCommunityCommand();
				break;
		}

		if (m_autonomousCommand != null) 
		{
			setToCoast();
			m_autonomousCommand.schedule();
		}
	}

	/** This function is called once when teleop is enabled. */
	@Override
	public void teleopInit() 
	{
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.

		System.out.println("********** TELEOPERATED STARTED ************");

		if (m_autonomousCommand != null)
			m_autonomousCommand.cancel();

		startTimer();
		reset();
		setToCoast();
	}

	/** This function is called once when the robot is disabled. */
	@Override
	public void disabledInit() 
	{
		System.out.println("********** ROBOT DISABLED ************");
		stopTimer();

		//m_robotAcquisition.compressorDisable();

		// disabledStarted();
	}

	/** This function is called once when test mode is enabled. */
	@Override
	public void testInit() 
	{
		System.out.println("********** TEST STARTED ************");
		// Cancels all running commands at the start of test mode.
		// CommandScheduler.getInstance().cancelAll();
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by
	 * instantiating a {@link GenericHID} or one of its subclasses ({@link
	 * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
	 * it to a {@link
	 * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
	 */
	private void configureDriverButtonBindings() 
	{
		SmartDashboard.putNumber("MAXSPEED", 0);

		new JoystickButton(m_driverController, Button.kRightBumper.value)
			.whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_TRIGGER_SPEED)))
			.onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED)));

		new JoystickButton(m_driverController, Button.kLeftBumper.value)
			.whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(SmartDashboard.getNumber("MAXSPEED", 0))))
			.onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED)));

		// new JoystickButton(_driverController, Button.kA.value).onTrue(new
		// InstantCommand(() -> _pigeon.reset()));

		// Stabilize robot to drive straight with gyro when left bumper is held
		// new JoystickButton(m_operatorController, Button.kLeftBumper.value)
		// .whileTrue(
		// new PIDCommand(
		// new PIDController(
		// DriveConstants.kStabilizationP,
		// DriveConstants.kStabilizationI,
		// DriveConstants.kStabilizationD),
		// // Close the loop on the turn rate
		// m_robotDrive::getTurnRate,
		// // Setpoint is 0
		// 0,
		// // Pipe the output to the turning controls
		// output -> m_robotDrive.arcadeDrive(-m_operatorController.getLeftY(), output),
		// // Require the robot drive
		// m_robotDrive));

		// // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
		new JoystickButton(m_driverController, Button.kX.value)
			.onTrue(new TurnRight(90, m_robotDrive).withTimeout(20));

		new JoystickButton(m_driverController, Button.kB.value)
			.onTrue(new TurnPID(m_pid, -90, m_robotDrive).withTimeout(5));

		new JoystickButton(m_driverController, Button.kY.value)
			.toggleOnTrue(new InstantCommand(() -> m_robotDrive.applyBrakesEndGame()));



		// .onFalse(new InstantCommand(() -> m_robotDrive.setToCoast()));

		// // Turn to -90 degrees with a profile when the Circle button is pressed, with
		// a
		// // 5 second timeout
		// new JoystickButton(_driverController, Button.kB.value)
		// .onTrue(new TurnToAngleProfiled(-90, _robotDrive).withTimeout(5));

		// new JoystickButton(m_operatorController, Button.kY.value)
		// .whileTrue(new BalanceCmd(m_robotDrive));

		// // new JoystickButton(m_operatorController, Button.kA.value)
		// // .onTrue(new DriveToPitch(m_robotDrive, .5, 1));
	}

	private void configureOperatorButtons() 
	{
		// new JoystickButton(m_operatorController, Button.kY.value)
		// .whileTrue(new Elevate(m_robotAcquisition));

		m_operatorController.rightTrigger(0.5, m_controllerEventLoop)
			.ifHigh(() -> m_robotAcquisition.grabberClose());

		//m_operatorController.a(m_controllerEventLoop).and(m_operatorController.b(m_controllerEventLoop));

		m_operatorController.leftTrigger(0.5, m_controllerEventLoop)
			.ifHigh(() -> m_robotAcquisition.grabberOpen());

		m_operatorController.rightTrigger(0.5, m_controllerEventLoop)
			.ifHigh(() -> m_robotAcquisition.grabberClose());

		new JoystickButton(m_operatorController, Button.kA.value)
			.onTrue(new ReturnToHome(m_robotAcquisition));

		new JoystickButton(m_operatorController, Button.kX.value)
			.onTrue(new MoveTo(m_robotAcquisition, MID_CUBE_Y, MID_CUBE_X));

		new JoystickButton(m_driverController, Button.kB.value)
			.onTrue(new MoveTo(m_robotAcquisition, MID_CONE_Y, MID_CONE_X));

		m_operatorController.leftBumper(m_controllerEventLoop).and(m_operatorController.b(m_controllerEventLoop))
			.ifHigh(() -> new MoveTo(m_robotAcquisition, HIGH_CONE_Y, HIGH_CONE_X));

		m_operatorController.rightBumper(m_controllerEventLoop).and(m_operatorController.x(m_controllerEventLoop))
			.ifHigh(() -> new MoveTo(m_robotAcquisition, HIGH_CUBE_X, HIGH_CUBE_Y));

		new JoystickButton(m_operatorController, Button.kY.value)
			.onTrue(new MoveTo(m_robotAcquisition, SHELF_X, SHELF_Y));
		
	

					/*
		Need 6 buttons:
		- Score high cone
		- Score low cone
		- Score high cube
		- Score low cube
		- Pick up from shelf
		- Open/close grabber
		*/
	}

	// /**
	// * @param left
	// * @param right
	// * @param speed
	// 
	// public void tankDrive(double left, double right, double speed) {
	// m_robotDrive.setMaxOutput(speed);
	// m_robotDrive.tankDrive(left, right);
	// }

	public double getPitch() 
	{
		return m_pigeon.getPitch();
	}

	public Command getShortAutoCommand() 
	{
		return new BalanceShortRobot(m_robotDrive);
	}

	public Command getScoreCommunityCommand() 
	{
		return new ScoreCommunity(m_robotDrive, m_robotAcquisition);
	}

	public double getTimerSeconds() 
	{
		return m_Timer.get();
	}

	public void startTimer() 
	{
		m_Timer.start();
	}

	public void restartTimer() 
	{
		m_Timer.restart();
	}

	public void stopTimer() 
	{
		m_Timer.stop();
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getLongAutoCommand() 
	{
		// return new SequentialCommandGroup(
		// new DriveDistance(m_robotDrive, 2, .8),
		// new DriveToPitch(m_robotDrive, .2, 1),
		// new DriveToPitch(m_robotDrive, .2, -1)
		// );

		// return new AutoDistances(m_robotDrive);

		return new BalanceLongRobot(m_robotDrive);
		// An ExampleCommand will run in autonomous
	}

	public Command getDriveMeasurements() 
	{
		return new DriveMeasurements(m_robotDrive);
	}

	public DriveSubsystem getDrives() 
	{
		return m_robotDrive;
	}

	public void applyBrakes() 
	{
		m_robotDrive.applyBrakes();
	}

	public void setToCoast() 
	{
		m_robotDrive.setToCoast();
	}

	public void reset() 
	{
		m_robotDrive.reset();
	}

	public void setTurnPID(PIDController pid) 
	{
		m_pid = pid;
	}

	public PIDController getTurnPID() 
	{
		m_pid = new PIDController(0, 0, 0);
		return m_pid;
	}
}
