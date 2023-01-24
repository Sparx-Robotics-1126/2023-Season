package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.commands.ReportingCommand;
import frc.subsystem.Pigeon2Subsystem;
import frc.subsystem.ReportingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.subsystem.DriveSubsystem;
import frc.subsystem.Drives;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Pigeon2Subsystem m_pigeon2subsystem = new Pigeon2Subsystem(Constants.Pigeon2ID, "rio");
  private final ReportingSubsystem m_reportingSubsystem = new ReportingSubsystem();

  private final XboxController m_joy = new XboxController(0);

  private final ReportingCommand m_reportingCommand = new ReportingCommand(m_reportingSubsystem, m_pigeon2subsystem);
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
//   public DifferentialDrive m_myRobot;
  private final XboxController m_driverController = new XboxController(0);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    m_reportingSubsystem.setDefaultCommand(m_reportingCommand);
    configureButtonBindings();
    // var drives = new Drives();

    //   m_myRobot = new DifferentialDrive(drives.LeftMotorMaster, drives.RightMotorMaster);
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
            () ->
                m_robotDrive.tankDriveVolts(
                    -m_driverController.getLeftY(), -m_driverController.getRightX()),
            m_robotDrive));
 

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
     new JoystickButton(m_driverController, Button.kRightBumper.value)
.onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)))
.onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
  }
private Command getTeleopCommand(){
    return m_reportingCommand;
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_reportingCommand;
  }
}