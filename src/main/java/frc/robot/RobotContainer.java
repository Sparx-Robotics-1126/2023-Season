package frc.robot;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import frc.robot.commands.ReportingCommand;
// import frc.subsystem.Pigeon2Subsystem;
// import frc.subsystem.ReportingSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.subsystem.DriveSubsystem;
import frc.subsystem.Drives;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.WPI_Pigeon2;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
 private final WPI_Pigeon2 _pigeon2; 
  // private final ReportingSubsystem m_reportingSubsystem = new ReportingSubsystem();

  // private final XboxController m_joy = new XboxController(0);

  // private final ReportingCommand m_reportingCommand = new ReportingCommand(m_reportingSubsystem);//, m_pigeon2subsystem);
  private final DriveSubsystem m_robotDrive;// = new DriveSubsystem();
//   public DifferentialDrive m_myRobot;
  private final XboxController m_driverController = new XboxController(0);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // m_reportingSubsystem.setDefaultCommand(m_reportingCommand);
  


    _pigeon2  = new WPI_Pigeon2(Constants.Pigeon2ID);
    InitPigeon();

    configureButtonBindings();
    // var drives = new Drives();
    m_robotDrive = new DriveSubsystem(_pigeon2);
    //   m_myRobot = new DifferentialDrive(drives.LeftMotorMaster, drives.RightMotorMaster);
    m_robotDrive.setDefaultCommand(
        
        new RunCommand(
            () ->  m_robotDrive.tankDrive(
                    -(m_driverController.getLeftY()) 
                     , m_driverController.getRightY()),
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
.onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(.5)))
.onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(1)));
new JoystickButton(m_driverController, Button.kA.value).onTrue(new InstantCommand(() -> 
InitPigeon()));


  }

  private void InitPigeon(){
     var toApply = new Pigeon2Configuration();
     _pigeon2.configAllSettings(toApply);
    // _pigeon2.getConfigurator().apply(toApply);

    /* Speed up signals to an appropriate rate */
    // _pigeon2.getYaw().setUpdateFrequency(100);
    // _pigeon2.getPitch().setUpdateFrequency(100);
    _pigeon2.getYaw();
    _pigeon2.getPitch();
// _pigeon2.setStatusFramePeriod(0,100 )
    // _pigeon2.getGravityVectorZ().setUpdateFrequency(100);
  }

  public double getPitch(){
   return  _pigeon2.getPitch();
  }
// private Command getTeleopCommand(){
//     return m_reportingCommand;
// }
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  //   // An ExampleCommand will run in autonomous
  //   return m_reportingCommand;
  // }

  // public DriveSubsystem getDrives(){
  //   return m_robotDrive;
  // }
}