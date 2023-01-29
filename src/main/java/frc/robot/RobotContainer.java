package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TurnToAngleProfiled;
import frc.robot.commands.TurnToAngle;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PigeonSubsystem;

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
  private final DriveSubsystem _robotDrive;
  private final XboxController _driverController;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    _driverController = new XboxController(Constants.XBOX_CONTROLLER_PORT);

    _pigeon = new PigeonSubsystem();

    configureButtonBindings();
    _robotDrive = new DriveSubsystem(_pigeon);
    _robotDrive.setDefaultCommand(

        new RunCommand(
            () -> _robotDrive.tankDrive(
                -(_driverController.getLeftY()), _driverController.getRightY()),
            _robotDrive));

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
    new JoystickButton(_driverController, Button.kRightBumper.value)
        .onTrue(new InstantCommand(() -> _robotDrive.setMaxOutput(.5)))
        .onFalse(new InstantCommand(() -> _robotDrive.setMaxOutput(1)));

    // new JoystickButton(_driverController, Button.kA.value).onTrue(new
    // InstantCommand(() -> _pigeon.reset()));

    // Stabilize robot to drive straight with gyro when left bumper is held
    new JoystickButton(_driverController, Button.kLeftBumper.value)
        .whileTrue(
            new PIDCommand(
                new PIDController(
                    DriveConstants.kStabilizationP,
                    DriveConstants.kStabilizationI,
                    DriveConstants.kStabilizationD),
                // Close the loop on the turn rate
                _robotDrive::getTurnRate,
                // Setpoint is 0
                0,
                // Pipe the output to the turning controls
                output -> _robotDrive.tankDrive(-_driverController.getLeftY(), output),
                // Require the robot drive
                _robotDrive));

    // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
    new JoystickButton(_driverController, Button.kX.value)
        .onTrue(new TurnToAngle(90, _robotDrive).withTimeout(5));

    // Turn to -90 degrees with a profile when the Circle button is pressed, with a
    // 5 second timeout
    new JoystickButton(_driverController, Button.kA.value)
        .onTrue(new TurnToAngleProfiled(-90, _robotDrive).withTimeout(5));

  }

  public double getPitch() {
    return _pigeon.getPitch();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // // An ExampleCommand will run in autonomous
  // return m_reportingCommand;
  // }

}