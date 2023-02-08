package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import frc.drives.commands.DriveForward;
import frc.robot.Constants.DriveConstants;
// import frc.robot.commands.TurnToAngleProfiled;
// import frc.robot.commands.Autonomous;
import frc.robot.commands.BalanceCmd;
import frc.robot.commands.DriveDistance;
// import frc.robot.commands.TurnToAngle;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PigeonSubsystem;
// import frc.robot.commands.DriveDistance;
import frc.robot.commands.DriveForwardCmd;
import frc.robot.commands.DriveToPitch;
// import frc.robot.commands.DriveToPitch;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import com.ctre.phoenix.sensors.BasePigeonSimCollection;
// import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
// import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
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
    private final XboxController m_driverController;

    private double driveSpeed = 0.9;
    private double turnSpeed = 0.8;
    private double triggerSpeed = 0.1; 
  

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        m_driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);

        _pigeon = new PigeonSubsystem();

        configureButtonBindings();
        
        m_robotDrive = new DriveSubsystem(_pigeon);
        m_robotDrive.setMaxOutput(driveSpeed);
        m_robotDrive.setDefaultCommand(

                new RunCommand(
                        () -> m_robotDrive.tankDrive(
                                -(m_driverController.getLeftY()), m_driverController.getRightY()),
                        m_robotDrive));

    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        new JoystickButton(m_driverController, Button.kRightBumper.value)
                .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(triggerSpeed)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(driveSpeed)));

        // new JoystickButton(_driverController, Button.kA.value).onTrue(new
        // InstantCommand(() -> _pigeon.reset()));

        // Stabilize robot to drive straight with gyro when left bumper is held
        new JoystickButton(m_driverController, Button.kY.value)
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
                                output -> m_robotDrive.arcadeDrive(-m_driverController.getLeftY(), output),
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

        // new JoystickButton(m_driverController, Button.kA.value)
        // .onTrue(new BalanceCmd(m_robotDrive));

        new JoystickButton(m_driverController, Button.kA.value)
        .onTrue(new DriveToPitch(m_robotDrive,.5,1));


    }

    public void tankDrive(double left, double right, double speed){
        m_robotDrive.setMaxOutput(speed);
        m_robotDrive.tankDrive(left, right);
    }

    public double getPitch() {
        return _pigeon.getPitch();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return new SequentialCommandGroup(  new DriveDistance(m_robotDrive, 5,.5));

       // return new SequentialCommandGroup(  new DriveToPitch(_robotDrive, .5),
                                            // new DriveToPitch(_robotDrive, -.5));
        
        
                                            // An ExampleCommand will run in autonomous

        // return new DriveForward(_robotDrive.getDriveSenors(),.15, 12);
        // return new DriveDistance(12, .2, _robotDrive);
        // return new Autonomous(_robotDrive);
    }

    public DriveSubsystem getDrives() {
        return m_robotDrive;
    }

}