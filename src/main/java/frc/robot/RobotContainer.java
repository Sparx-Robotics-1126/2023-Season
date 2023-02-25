package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.BalanceShortRobot;
import frc.robot.commands.BalanceLongRobot;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.commands.DriveMeasurements;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.PigeonSubsystem;
import frc.robot.commands.Elevate;
import frc.sensors.Limelight;

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
    private final PigeonSubsystem _pigeon;
    // BasePigeonSimCollection m_pigeonSim = _pigeon.getSimCollection();
    private final DriveSubsystem m_robotDrive;
    private final AcquisitionSubsystem m_robotAcquisition;
    private final XboxController m_driverController;
    private final XboxController m_operatorController;

    private double driveSpeed = 0.9;
    private double triggerSpeed = 0.1;

    private Limelight m_limeLight;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        m_limeLight = new Limelight();
        m_limeLight.enableVision();
        
        m_driverController = new XboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT);
        m_operatorController = new XboxController(Constants.XBOX_OPERATOR_CONTROLLER_PORT);
        _pigeon = new PigeonSubsystem();

        m_robotAcquisition = new AcquisitionSubsystem();
        
        m_robotDrive = new DriveSubsystem(_pigeon);
        m_robotDrive.setMaxOutput(driveSpeed);
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.tankDrive(
                                (m_driverController.getLeftY()), m_driverController.getRightY()),
                        m_robotDrive));

        // m_robotDrive.setDefaultCommand(

        // new PIDCommand(
        // joystickPID,
        // m_robotDrive::getEncoderMeters,
        // output -> m_robotDrive.tankDrive(m_driverController.getLeftY()),
        // m_driverController.getRightY()),
        // m_robotDrive);
        configureDriverButtonBindings();
        configureOperatorButtons();

        // m_robotAcquisition.setDefaultCommand(

        // new RunCommand(
        // () -> m_robotAcquisition.elevate()

        // ));

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
        new JoystickButton(m_operatorController, Button.kRightBumper.value)
                .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(triggerSpeed)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(driveSpeed)));

        // new JoystickButton(_driverController, Button.kA.value).onTrue(new
        // InstantCommand(() -> _pigeon.reset()));

        // Stabilize robot to drive straight with gyro when left bumper is held
        new JoystickButton(m_operatorController, Button.kLeftBumper.value)
                .whileTrue(
                        new PIDCommand(
                                new PIDController(
                                        DriveConstants.kStabilizationP,
                                        DriveConstants.kStabilizationI,
                                        DriveConstants.kStabilizationD),
                                // Close the loop on the turn rate
                                m_robotDrive::getTurnRate,
                                // Setpoint is 0
                                0,
                                // Pipe the output to the turning controls
                                output -> m_robotDrive.arcadeDrive(-m_operatorController.getLeftY(), output),
                                // Require the robot drive
                                m_robotDrive));

        // // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
        // new JoystickButton(_driverController, Button.kX.value)
        // .onTrue(new TurnToAngle(90, _robotDrive).withTimeout(5));

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
        new JoystickButton(m_operatorController, Button.kY.value)
                .whileTrue(new Elevate(m_robotAcquisition));
    }



    /**
     * @param left
     * @param right
     * @param speed
     */
    public void tankDrive(double left, double right, double speed) {
        m_robotDrive.setMaxOutput(speed);
        m_robotDrive.tankDrive(left, right);
    }

    public double getPitch() {
        return _pigeon.getPitch();
    }

public Command getShortAutoCommand(){
    return new BalanceShortRobot(m_robotDrive);

}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getLongAutoCommand() {

        
        // return new SequentialCommandGroup(
        // new DriveDistance(m_robotDrive, 2, .8),
        // new DriveToPitch(m_robotDrive, .2, 1),
        // new DriveToPitch(m_robotDrive, .2, -1)
        // );

        //  return new AutoDistances(m_robotDrive);

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