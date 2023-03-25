package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.Constants.DriveConstants;
import frc.robot.commands.Acquisition.MoveTo;
// import frc.robot.commands.Autonomous.Autos;
import frc.robot.commands.Autonomous.BalanceLongRobot;
import frc.robot.commands.Autonomous.BalanceShortRobot;
import frc.robot.commands.Autonomous.LongComm;
import frc.robot.commands.Autonomous.ScoreCommunity;
import frc.robot.commands.Autonomous.ShortComm;
import frc.robot.commands.Drive.DriveDistance;
import frc.robot.commands.Drive.DriveDistanceCmd;
import frc.robot.commands.Drive.DriveMeasurements;
import frc.robot.subsystem.AcquisitionSubsystem;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.PigeonSubsystem;
import frc.robot.subsystem.ShuffleSubsystem;
import frc.robot.sensors.Limelight;
import static frc.robot.Constants.FieldConstants.*;

import java.util.ArrayList;

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
    // private final AcquisitionSubsystem m_robotAcquisition;
    private final CommandXboxController m_driverController;
    private final CommandXboxController m_operatorController;
    private final XboxController m_xboxController;

    private boolean slowSpeedEnabled;
    private boolean mediumSpeedEnabled;
    private boolean fullSpeedEnabled;

    private boolean isSimulation;

    private ArrayList<ShuffleSubsystem> m_subsystems;
    private int outputCounter;
    private Notifier updateNotifier;

    private ArrayList<ShuffleSubsystem> subsystems;

    private interface CommandSupplier {
        Command getCommand();
    }

    private final SendableChooser<CommandSupplier> _chooser = new SendableChooser<>();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        Limelight limelight = new Limelight();

        m_driverController = new CommandXboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT);
        m_operatorController = new CommandXboxController(Constants.XBOX_OPERATOR_CONTROLLER_PORT);
        m_pigeon = PigeonSubsystem.getInstance();
        m_xboxController = new XboxController(Constants.XBOX_DRIVER_CONTROLLER_PORT1);

        // m_robotAcquisition = new AcquisitionSubsystem();

        m_robotDrive = new DriveSubsystem();
        m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED);
        m_robotDrive.setDefaultCommand(
                new RunCommand(
                        () -> m_robotDrive.tankDrive(
                                (m_driverController.getLeftY()), m_driverController.getRightY()),
                        m_robotDrive));

        configureDriverButtonBindings();
        // configureOperatorButtons();
        configureChooser();
        configureShuffleboard();

        updateNotifier = new Notifier(this::update);
        updateNotifier.startPeriodic(Constants.UPDATE_PERIOD);

        isSimulation = RobotBase.isSimulation();

        subsystems = new ArrayList<ShuffleSubsystem>();
        // add each of the subsystems to the arraylist here

        subsystems.add(m_robotDrive);
        // subsystems.add(vision);
        // subsystems.add(m_robotAcquisition);

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
        // SmartDashboard.putNumber("MAXSPEED", 0);
        // SmartDashboard.putBooleanArray("Speed Toggles", new boolean[]
        // {slowSpeedEnabled, mediumSpeedEnabled, fullSpeedEnabled});

        m_driverController.rightBumper()
                .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_RIGHT_TRIGGER_SPEED)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED)));

        m_driverController.leftBumper()
                .whileTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_LEFT_TRIGGER_SPEED)))
                .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(DriveConstants.MAX_DRIVE_SPEED)));

        // // Turn to 90 degrees when the 'X' button is pressed, with a 5 second timeout
        // m_driverController.b()
        // .onTrue(new TurnRight(90, m_robotDrive).withTimeout(20));

        m_driverController.y()
                .toggleOnTrue(new InstantCommand(() -> m_robotDrive.applyBrakesEndGame()));

        // new JoystickButton(m_operatorController, Button.kA.value)
        // .onTrue(new MoveTo(m_robotAcquisition, 0, 0.5));

        m_driverController.a()
                .whileTrue(new InstantCommand(() -> m_xboxController.setRumble(GenericHID.RumbleType.kBothRumble, .5)))
                .onFalse(new InstantCommand(() -> m_xboxController.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

    }

    private void configureOperatorButtons() {

        // // Grabber Open
        // m_operatorController.leftTrigger()
        // .onTrue(new InstantCommand(() -> m_robotAcquisition.grabberOpen()));

        // // Grabber Closed
        // m_operatorController.rightTrigger()
        // .onTrue(new InstantCommand(() -> m_robotAcquisition.grabberClose()));

        // // Return To Home
        // m_operatorController.a()
        // .onTrue(new MoveTo(0, 0, m_robotAcquisition));

        // // Get from Human Shelf
        // m_operatorController.y()
        // .onTrue(new MoveTo(SHELF_X, SHELF_Y, m_robotAcquisition));

        // // Score Mid Cube
        // m_operatorController.x().and(m_operatorController.leftBumper().negate())
        // .onTrue(new MoveTo(MID_CUBE_X, MID_CUBE_Y, m_robotAcquisition));

        // // Score Mid Cone
        // m_operatorController.b().and(m_operatorController.leftBumper().negate())
        // .onTrue(new MoveTo(MID_CONE_X, MID_CONE_Y, m_robotAcquisition));

        // // Score Cube High
        // m_operatorController.x().and(m_operatorController.leftBumper())
        // .onTrue(new MoveTo(HIGH_CUBE_X, HIGH_CUBE_Y, m_robotAcquisition));

        // // Score Cone High
        // m_operatorController.b().and(m_operatorController.leftBumper())
        // .onTrue(new MoveTo(HIGH_CONE_X, HIGH_CONE_Y, m_robotAcquisition));

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

    public void configureChooser() {

        // _chooser.setDefaultOption("Short PID", () ->
        // Autos.balanceChargeStations(m_robotDrive));
        _chooser.addOption("Long", () -> new BalanceLongRobot(m_robotDrive));
        _chooser.setDefaultOption("Short", () -> new BalanceShortRobot(m_robotDrive));
        _chooser.addOption("Measure", () -> new DriveDistance(m_robotDrive, 6, .4).withTimeout(6));
        // _chooser.addOption("Score and Leave Community", () -> new
        // ScoreCommunity(m_robotDrive, m_robotAcquisition));
        _chooser.addOption("NewDistance", () -> new DriveDistanceCmd(m_robotDrive, 1, .75));
        _chooser.addOption("Short Comm", () -> new ShortComm(m_robotDrive, getAcquisition()));
        _chooser.addOption("Long Comm", () -> new LongComm(m_robotDrive, getAcquisition()));
        _chooser.addOption("Do Nothing", () -> new InstantCommand());

        SmartDashboard.putData("AUTO CHOICES ", _chooser);
    }

    public void configureShuffleboard() {
        // Field Side
        SmartDashboard.putBoolean("isAllianceBlue", getAllianceColor());
        // SmartDashboard.putBoolean("Testing", false);
        // getting the auto values for score, cargo, and charge
        // SmartDashboard.putBoolean("1st Auto Score", firstScore);
        // SmartDashboard.putBoolean("Opt. 2nd Auto Score", secondScore);
        // SmartDashboard.putBoolean("Auto Get Cargo", cargo);
        // SmartDashboard.putBoolean("Auto Goto Charge", charge);
        // SmartDashboard.putNumber("View Trajectory Pos", 0);
        SmartDashboard.putBoolean("Update Visual", false);
        // SmartDashboard.putBoolean("3 Ball Auto", false);
        SmartDashboard.putBoolean("Leave Tarmac", true);
        SmartDashboard.putBoolean("Hit and Run", false);

        SmartDashboard.putBoolean("Reset Auto Viewer", false);

    }

    private void setSpeedToggles(String speed) {
        slowSpeedEnabled = false;
        mediumSpeedEnabled = false;
        fullSpeedEnabled = false;

        switch (speed) {
            case "slow": {
                slowSpeedEnabled = true;
            }
            case "medium": {
                mediumSpeedEnabled = true;
            }
            case "full": {
                fullSpeedEnabled = true;
            }
            default: {
                fullSpeedEnabled = true;
            }
        }
    }

    public Command getAutonomousCommand() {

        return new InstantCommand(() -> m_robotDrive.resetPitch())
                .andThen(() -> m_robotDrive.setToCoast())
                .andThen(_chooser.getSelected().getCommand());
    }

    public double getPitch() {
        return m_pigeon.getPitch();
    }

    public CommandXboxController getOperatorController() {
        return m_operatorController;
    }

    public Command getShortAutoCommand() {
        return new BalanceShortRobot(m_robotDrive);
    }

    // public Command getScoreCommunityCommand() {
    // return new ScoreCommunity(m_robotDrive, m_robotAcquisition);
    // }
    public boolean getGrabberState() {
        return false;
        // return m_robotAcquisition.getGrabberState();
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
        return null;
        // return m_robotAcquisition;
    }

    private boolean getAllianceColor() {
        return DriverStation.getAlliance() == DriverStation.Alliance.Blue;
    }

    private static long t = System.currentTimeMillis() + 2000;

    public void EndGameRumble() {
        
     if (DriverStation.getMatchTime() < 28) {
          return;

     }

    

        if(DriverStation.getMatchTime() < DriveConstants.EndGameSeconds ) {
           
           while (System.currentTimeMillis() < t ){
            m_xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, .5);
            m_xboxController.setRumble(GenericHID.RumbleType.kRightRumble, .5);
            System.out.println("Rumble started"); 
           }

        } 
    }

    // public void EndEndGameRumble() {
    //     if(DriverStation.getMatchTime() > DriveConstants.StopRumble ) {
    //         m_xboxController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    //         m_xboxController.setRumble(GenericHID.RumbleType.kRightRumble,0);
    //         System.out.println("Rumble stopped");
    //     }
    // }

    /**
     * Update all of the subsystems
     * This is run in a separate loop at a faster rate to:
     * a) update subsystems faster
     * b) prevent packet delay from driver station from delaying response from our
     * robot
     */
    private void update() {
        for (ShuffleSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    public void displayShuffleboard() {

        if (m_subsystems == null || m_subsystems.size() == 0) {
            return;
        }

        if (outputCounter % 3 == 0) {
            m_subsystems.get(outputCounter / 3).displayShuffleboard();
        }

        PigeonSubsystem.getInstance().outputValues();
        tuningPeriodic();

        outputCounter = (outputCounter + 1) % (m_subsystems.size() * 3);

    }

    private void tuningPeriodic() {
        if (outputCounter % 3 == 0) {
            m_subsystems.get(outputCounter / 3).tuningPeriodic();
        }

        if (isSimulation && SmartDashboard.getBoolean("Reset Auto Viewer", false)) {
            // updateTraj = true;
            SmartDashboard.putBoolean("Reset Auto Viewer", false);
        }

        // if (updateTraj) { // change the trajectory drawn
        // // generateTrajedies.incrementOutputCounter();
        // Trajectory traj =
        // generateTrajectories.getTrajectory((int)SmartDashboard.getNumber("View
        // Trajectory Pos", 0));
        // if (traj != null)
        // drivetrain.drawTrajectory(traj);
        // }

        // if (updateTraj && checkIfUpdate()) {
        // DriverStation.reportWarning("Updating Auto", cargo);
        // updateAutoChoosers();

        // generateTrajectories = new GenerateTrajectories(
        // drivetrain,
        // charge,
        // firstScore,
        // secondScore,
        // cargo,
        // estimatedCurrentPose2d(),
        // threePiece,
        // leaveTarmac,
        // hitAndRun
        // );

        // SmartDashboard.putNumber("View Trajectory Pos",
        // generateTrajectories.getLastTrajectoryIndex());

        // putTrajectoryTime();
        // resetDashboard();
        // }
    }
}