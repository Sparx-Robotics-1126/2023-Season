package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Acquisition.MoveTo;
import frc.robot.commands.Acquisition.ReturnToHome;
import frc.robot.commands.Autonomous.BalanceLongRobot;
import frc.robot.commands.Autonomous.BalanceShortRobot;
import frc.robot.commands.Autonomous.ScoreCommunity;
import frc.robot.commands.Drive.DriveMeasurements;
import frc.robot.commands.Drive.TurnRight;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PigeonSubsystem;
import frc.robot.sensors.Limelight;
import static frc.robot.Constants.FieldConstants.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final PigeonSubsystem m_pigeon;
    private final DriveSubsystem m_robotDrive;
    private final AcquisitionSubsystem m_robotAcquisition;
    private final CommandXboxController m_driverController;
    private final CommandXboxController m_operatorController;

    private final Timer m_Timer;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_Timer = new Timer();
        Limelight limeLight = new Limelight();
        limeLight.enableVision();

        m_driverController = new CommandXboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT);
        m_operatorController = new CommandXboxController(Constants.XBOX_OPERATOR_CONTROLLER_PORT);
        m_pigeon = new PigeonSubsystem();

        m_robotAcquisition = new AcquisitionSubsystem();

        m_robotDrive = new DriveSubsystem(m_pigeon, m_Timer);
        m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED);
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.tankDrive(
                                (m_driverController.getLeftY()), m_driverController.getRightY()),
                        m_robotDrive));

        configureDriverButtonBindings();
        configureOperatorButtons();

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureDriverButtonBindings() {
        SmartDashboard.putNumber("MAXSPEED", 0);

        m_driverController.rightBumper()
                .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_TRIGGER_SPEED)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED)));

        m_driverController.leftBumper()
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
        m_driverController.b()
                .onTrue(new TurnRight(90, m_robotDrive).withTimeout(20));

        m_driverController.y()
                .toggleOnTrue(new InstantCommand(() -> m_robotDrive.applyBrakesEndGame()));

        // new JoystickButton(m_operatorController, Button.kA.value)
        // .onTrue(new MoveTo(m_robotAcquisition, 0, 0.5));

        /*
         * Need 6 buttons:
         * - Score high cone
         * - Score low cone
         * - Score high cube
         * - Score low cube
         * - Pick up from shelf
         * - Open/close grabber
         */

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

    private void configureOperatorButtons() {

        // Grabber Open
        m_operatorController.leftTrigger()
                .onTrue(new InstantCommand(() -> m_robotAcquisition.grabberOpen()));

        // Grabber Closed
        m_operatorController.rightTrigger()
                .onTrue(new InstantCommand(() -> m_robotAcquisition.grabberClose()));

        // Return To Home
        m_operatorController.a()
                .onTrue(new ReturnToHome(m_robotAcquisition));

        // Get from Human Shelf
        m_operatorController.y()
                .onTrue(new MoveTo(m_robotAcquisition, SHELF_X, SHELF_Y));

        // Score Mid Cube
        m_operatorController.x().and(m_operatorController.leftBumper().negate())
                .onTrue(new MoveTo(m_robotAcquisition, MID_CUBE_X, MID_CUBE_Y));

        // Score Mid Cone
        m_operatorController.b().and(m_operatorController.leftBumper().negate())
                .onTrue(new MoveTo(m_robotAcquisition, MID_CONE_X, MID_CONE_Y));

        // Score Cube High
        m_operatorController.x().and(m_operatorController.leftBumper())
                .onTrue(new MoveTo(m_robotAcquisition, HIGH_CUBE_X, HIGH_CUBE_Y));

        // Score Cone High
        m_operatorController.b().and(m_operatorController.leftBumper())
                .onTrue(new MoveTo(m_robotAcquisition, HIGH_CONE_X, HIGH_CONE_Y));

        // m_operatorController.a(m_controllerEventLoop).and(m_operatorController.b(m_controllerEventLoop));

        /*
         * Need 6 buttons:
         * - Score high cone - done
         * - Score mid cone - done
         * - Score low cone
         * - Score high cube - done
         * - Score mid cube - done
         * - Score low cube
         * - Pick up from shelf - done
         * - Open/close grabber - done
         */
    }

    // /**
    // * @param left
    // * @param right
    // * @param speed
    // */
    // public void tankDrive(double left, double right, double speed) {
    // m_robotDrive.setMaxOutput(speed);
    // m_robotDrive.tankDrive(left, right);
    // }

    public double getPitch() {
        return m_pigeon.getPitch();
    }

    public Command getShortAutoCommand() {
        return new BalanceShortRobot(m_robotDrive);
    }

    public Command getScoreCommunityCommand() {
        return new ScoreCommunity(m_robotDrive, m_robotAcquisition);
    }

    public double getTimerSeconds() {
        return m_Timer.get();
    }

    public void startTimer() {
        m_Timer.start();
    }

    public void restartTimer() {
        m_Timer.restart();
    }

    public void stopTimer() {
        m_Timer.stop();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getLongAutoCommand() {

        return new BalanceLongRobot(m_robotDrive);
        // An ExampleCommand will run in autonomous
    }

    public Command getDriveMeasurements() {

        return new DriveMeasurements(m_robotDrive);
    }

    public DriveSubsystem getDrives() {
        return m_robotDrive;
    }

    public void applyBrakes() {
        m_robotDrive.applyBrakes();
    }

    public void setToCoast() {
        m_robotDrive.setToCoast();
    }

    public void reset() {
        m_robotDrive.reset();
    }

    public AcquisitionSubsystem getAcquisition() {
        return m_robotAcquisition;
    }
}