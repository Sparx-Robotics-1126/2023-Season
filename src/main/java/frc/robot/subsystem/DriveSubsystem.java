package frc.robot.subsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DriveConstants.*;

import java.sql.Driver;

public class DriveSubsystem extends ShuffleSubsystem {

  // Put motor initialization here.
  private CANSparkMax m_rightMotor;
  private CANSparkMax m_leftMotor;
  private CANSparkMax m_rightMotorSlave;
  private CANSparkMax m_leftMotorSlave;

  // The left-side drive encoder
  private RelativeEncoder m_leftEncoder;
  // // The right-side drive encoder
  private RelativeEncoder m_rightEncoder;

  // The robot's drive
  public DifferentialDrive m_driveDifferential;

  private SparkMaxPIDController m_leftPIDController;
  private SparkMaxPIDController m_rightPIDController;

  private PIDController m_distancePIDController;
  private PIDController m_anglePIDController;

  private TrapezoidProfile m_distanceProfile;
  // private RamseteController ramseteController;
  // private Trajectory m_trajectory;
  private Timer m_pathTimer; // measures how far along we are on our current profile / trajectory

  private final PIDController m_leftPID = new PIDController(kPDriveVel, 0, 0);
  private final PIDController m_rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  // The gyro sensor
  // private final PigeonSubsystem m_pigeon;// = new
  // WPI_Pigeon2(Constants.Pigeon2ID);

  private DifferentialDriveKinematics m_driveKinematics;
  private DifferentialDrivePoseEstimator m_poseEstimator;

  private DifferentialDriveOdometry m_odometry;
  private Boolean m_brakesOn;

  private double m_distSetpoint; // current distance setpoint in meters
  private double m_angleSetpoint; // current angle setpoint in deg

  // default value for when the setpoint is not set. Deliberately set low to avoid
  // skewing graphs
  private final double m_defaultSetpoint = -1.257;

  private double m_pitchOffset = 0;

  private boolean slowSpeedEnabled;
  private boolean mediumSpeedEnabled;
  private boolean fullSpeedEnabled;

  public enum State {
    TANK_DRIVE,
    ARCADE_DRIVE,
    // TURN_TRACK,
    // VELOCITY_DRIVE,
    DRIVE_DIST,
    TURN_ANGLE
    // DRIVE_DIST_PROFILED,
    // TRAJECTORY
  }

  private double m_speedLeft; // used for tank drive
  private double m_speedRight; // used for tank drive
  private double m_arcadeSpeed;
  private double m_turnAngle;

  private State m_defaultState = State.TANK_DRIVE;
  private State m_state = m_defaultState; // stores the current driving mode of the drivetrain

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // stop();
    // m_pigeon = pigeon;

    // configureMotors();
    // configureEncoders();
    // configurePID();
    // configureDifferentialDrive();

    m_rightMotor = new CANSparkMax(DriveConstants.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
    m_rightMotorSlave = new CANSparkMax(DriveConstants.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);

    m_leftMotor = new CANSparkMax(DriveConstants.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
    m_leftMotorSlave = new CANSparkMax(DriveConstants.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);

    // configureMotor(m_rightMotor, m_rightMotorSlave);
    // configureMotor(m_leftMotor, m_leftMotorSlave);
    // configurePID();

    m_leftMotor.setInverted(true);
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder
        .setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio / 60.0);
    m_rightEncoder
        .setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio / 60.0);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    // configure velocity PID controllers
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();

    m_leftPIDController.setP(DRIVE_VEL_LEFT_P, DRIVE_VEL_SLOT);
    m_leftPIDController.setFF(DRIVE_VEL_LEFT_F, DRIVE_VEL_SLOT);
    m_rightPIDController.setP(DRIVE_VEL_RIGHT_P, DRIVE_VEL_SLOT);
    m_rightPIDController.setFF(DRIVE_VEL_RIGHT_F, DRIVE_VEL_SLOT);

    // configure drive distance PID controllers
    m_distancePIDController = new PIDController(DRIVE_DIST_PID[0], DRIVE_DIST_PID[1], DRIVE_DIST_PID[2], UPDATE_PERIOD);
    m_distancePIDController.setTolerance(DRIVE_DIST_TOLERANCE); // TODO Look into velocity tolerance as well

    // // configure turn angle PID controllers
    m_anglePIDController = new PIDController(DRIVE_ANGLE_PID[0], DRIVE_ANGLE_PID[1], DRIVE_ANGLE_PID[2], UPDATE_PERIOD);
    m_anglePIDController.setTolerance(DRIVE_ANGLE_TOLERANCE); // TODO Look into velocity tolerance as well
    m_anglePIDController.enableContinuousInput(-180.0, 180.0);

    // m_driveDifferential.setDeadband(Constants.DEAD_BAND);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // _rightMotors.setInverted(true);

    // m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    // m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);

    // Burn settings into Spark MAX flash
    m_rightMotor.burnFlash();
    m_rightMotorSlave.burnFlash();
    m_leftMotor.burnFlash();
    m_leftMotorSlave.burnFlash();

    m_driveDifferential = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // // Set drive deadband and safety
    m_driveDifferential.setDeadband(0.05);
    m_driveDifferential.setSafetyEnabled(true);
    m_driveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

    // // var leftConversionFactor =
    // m_leftMotor.getEncoder().getPositionConversionFactor();
    // // var rightConversionFactor =
    // m_rightMotor.getEncoder().getPositionConversionFactor();
    // // var test = m_leftMotor.getEncoder().getPositionConversionFactor();
    // m_poseEstimator = new DifferentialDrivePoseEstimator(m_driveKinematics,
    // Rotation2d.fromDegrees(-PigeonSubsystem.getInstance().getAngle()),
    // m_leftMotor.getEncoder().getPositionConversionFactor(),
    // m_rightMotor.getEncoder().getPositionConversionFactor(), new Pose2d(0, 0, new
    // Rotation2d(0.0)));

    m_brakesOn = false;
    m_pathTimer = new Timer();
  }

  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(), -m_rightEncoder.getPosition());

    SmartDashboard.putString("Drive State", m_state.name());
    SmartDashboard.putNumber("ROBOT_ANGLE", getHeading());

    SmartDashboard.putNumber("LEFT DIST", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("RIGHT DIST", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("AVG DIST", getAverageEncoderDistance());

    SmartDashboard.putNumber("LEFT_MOTOR_PERCENT", m_leftMotor.get());
    SmartDashboard.putNumber("RIGHT_MOTOR_PERCENT", m_rightMotor.get());

    SmartDashboard.putNumber("LEFT_MOTOR_VOLTAGE", m_leftMotor.getBusVoltage());
    SmartDashboard.putNumber("RIGHT_MOTOR_VOLTAGE", m_rightMotor.getBusVoltage());

    SmartDashboard.putNumber("LEFT_MOTOR_AMPS", m_leftMotor.getOutputCurrent());
    SmartDashboard.putNumber("RIGHT_MOTOR_AMPS", m_rightMotor.getOutputCurrent());
    SmartDashboard.putBoolean("Brakes", m_brakesOn);

    PigeonSubsystem.getInstance().outputValues();

    SmartDashboard.putNumber("Game Time", DriverStation.getMatchTime());

    SmartDashboard.putBoolean("Slow Enabled", slowSpeedEnabled);
    SmartDashboard.putBoolean("Medium Enabled", mediumSpeedEnabled);
    SmartDashboard.putBoolean("Full Enabled", fullSpeedEnabled);
  }

  /**
   * Configures motors to follow one controller.
   * 
   * @param master The controller to follow.
   * @param slaves The controllers that should follow master.
   */
  private static void configureMotor(CANSparkMax master, CANSparkMax... slaves) {
    master.restoreFactoryDefaults();
    master.set(0);
    master.setIdleMode(IdleMode.kCoast);
    master.enableVoltageCompensation(Constants.NOMINAL_VOLTAGE);
    master.setSmartCurrentLimit(Constants.MAX_CURRENT);
    // master.setOpenLoopRampRate(1); //used for demo

    for (CANSparkMax slave : slaves) {
      slave.restoreFactoryDefaults();
      slave.follow(master);
      slave.setIdleMode(IdleMode.kCoast);
      slave.setSmartCurrentLimit(Constants.MAX_CURRENT);
      // slave.setOpenLoopRampRate(1); //used for demo
    }
  }

  private void configureMotors() {
    m_rightMotor = new CANSparkMax(DriveConstants.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
    m_rightMotorSlave = new CANSparkMax(DriveConstants.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);

    m_leftMotor = new CANSparkMax(DriveConstants.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
    m_leftMotorSlave = new CANSparkMax(DriveConstants.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);

    configureMotor(m_rightMotor, m_rightMotorSlave);
    configureMotor(m_leftMotor, m_leftMotorSlave);

    m_leftMotor.setInverted(true);
    // Burn settings into Spark MAX flash
    m_rightMotor.burnFlash();
    m_rightMotorSlave.burnFlash();
    m_leftMotor.burnFlash();
    m_leftMotorSlave.burnFlash();
  }

  private void configureEncoders() {
    m_leftEncoder = m_leftMotor.getEncoder();
    m_rightEncoder = m_rightMotor.getEncoder();

    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setInverted(true);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder
        .setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio / 60.0);
    m_rightEncoder
        .setVelocityConversionFactor(Math.PI * DriveConstants.kWheelDiameterMeters / DriveConstants.kGearRatio / 60.0);

    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  // configure all PID settings on the motors
  private void configurePID() {
    // configure velocity PID controllers
    m_leftPIDController = m_leftMotor.getPIDController();
    m_rightPIDController = m_rightMotor.getPIDController();

    m_leftPIDController.setP(DRIVE_VEL_LEFT_P, DRIVE_VEL_SLOT);
    m_leftPIDController.setFF(DRIVE_VEL_LEFT_F, DRIVE_VEL_SLOT);
    m_rightPIDController.setP(DRIVE_VEL_RIGHT_P, DRIVE_VEL_SLOT);
    m_rightPIDController.setFF(DRIVE_VEL_RIGHT_F, DRIVE_VEL_SLOT);

    // configure drive distance PID controllers
    m_distancePIDController = new PIDController(DRIVE_DIST_PID[0],
        DRIVE_DIST_PID[1], DRIVE_DIST_PID[2], UPDATE_PERIOD);
    m_distancePIDController.setTolerance(DRIVE_DIST_TOLERANCE); // TODO Look into velocity tolerance as well

    // configure turn angle PID controllers
    m_anglePIDController = new PIDController(DRIVE_ANGLE_PID[0],
        DRIVE_ANGLE_PID[1], DRIVE_ANGLE_PID[2], UPDATE_PERIOD);
    m_anglePIDController.setTolerance(DRIVE_ANGLE_TOLERANCE); // TODO Look into velocity tolerance as well
    m_anglePIDController.enableContinuousInput(-180.0, 180.0);
  }

  private void configureDifferentialDrive() {
    m_driveDifferential = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // Set drive deadband and safety
    m_driveDifferential.setDeadband(0.05);
    m_driveDifferential.setSafetyEnabled(true);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());
    m_driveKinematics = new DifferentialDriveKinematics(kTrackWidthMeters);

    // m_poseEstimator = new DifferentialDrivePoseEstimator(m_driveKinematics,
    // Rotation2d.fromDegrees(-PigeonSubsystem.getInstance().getAngle()),
    // m_leftMotor.getEncoder().getPositionConversionFactor(),
    // m_rightMotor.getEncoder().getPositionConversionFactor(), new Pose2d(0, 0, new
    // Rotation2d(0.0)));
  }

  /**
   * @return double
   */
  public double getEncoderMeters() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2 * DriveConstants.kEncoderTick2Meter;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getVelocity(), m_rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(
        PigeonSubsystem.getInstance().getRotation2d(), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition(), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftY  the commanded left output
   * @param rightY the commanded right output
   */
  public void tankDrive(double leftY, double rightY) {

    m_speedLeft = leftY;
    m_speedRight = rightY;
    m_defaultState = State.TANK_DRIVE;
    m_state = State.TANK_DRIVE;
    m_driveDifferential.tankDrive(leftY, rightY, true);
  }

  /**
   * @param leftVelocitySetpoint
   * @param rightVelocitySetpoint
   */
  public void tankDriveWithFeedforwardPID(double leftVelocitySetpoint, double rightVelocitySetpoint) {
    m_leftMotor.setVoltage(m_feedforward.calculate(leftVelocitySetpoint)
        + m_leftPID.calculate(m_leftEncoder.getVelocity(), leftVelocitySetpoint));
    m_rightMotor.setVoltage(m_feedforward.calculate(rightVelocitySetpoint)
        + m_rightPID.calculate(-m_rightEncoder.getVelocity(), rightVelocitySetpoint));
    m_driveDifferential.feed();
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double speed, double rot) {
    m_arcadeSpeed = speed;
    m_turnAngle = rot;
    m_defaultState = State.ARCADE_DRIVE;
    m_state = State.ARCADE_DRIVE;
    // m_driveDifferential.arcadeDrive(fwd, rot);
  }

  public void reset() {
    m_distanceProfile = null;
    // m_trajectory = null;
    m_pathTimer.stop();
    m_pathTimer.reset();

    zeroHeading();
    resetEncoders();
    // m_odometry.resetPosition(Rotation2d.fromDegrees(getHeading()),
    // m_leftEncoder.getPosition(),
    // m_rightEncoder.getPosition(), new Pose2d());

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    PigeonSubsystem.getInstance().reset();
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return m_leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return m_rightEncoder;
  }

  /**
   * @return The current distance that the left encoder is reporting.
   */

  public double getLeftEncoderDistance() {
    return m_leftEncoder.getPosition() * DriveConstants.ENCODER_MULTIPLIER;
  }

  /**
   * @return The current distance that the right encoder is reporting.
   */

  public double getRightEncoderDistance() {
    return m_rightEncoder.getPosition() * DriveConstants.ENCODER_MULTIPLIER;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_driveDifferential.setMaxOutput(maxOutput);
    slowSpeedEnabled = false;
    mediumSpeedEnabled = false;
    fullSpeedEnabled = false;

    if (DriveConstants.MAX_RIGHT_TRIGGER_SPEED == maxOutput) {
      slowSpeedEnabled = true;
      return;
    }
    if (DriveConstants.MAX_LEFT_TRIGGER_SPEED == maxOutput) {
      mediumSpeedEnabled = true;
      return;
    }
    fullSpeedEnabled = true;

  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    PigeonSubsystem.getInstance().reset();
  }

  public void driveDistance(double distance) {
    // driveDistance(distance, DRIVE_DIST_MAX_OUTPUT);
  }

  public void driveDistance(double distance, double overrideSpeed) {
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);
    PigeonSubsystem.getInstance().setYaw(0);

    m_distSetpoint = distance;
    m_angleSetpoint = 0;
    m_arcadeSpeed = overrideSpeed;
    m_distancePIDController.reset();

    m_distancePIDController.setSetpoint(m_distSetpoint);

    m_state = State.DRIVE_DIST;
  }

  public void turnAngle(double angle) {
    PigeonSubsystem.getInstance().reset();

    m_angleSetpoint = angle;
    m_anglePIDController.reset();

    m_pathTimer.reset();
    m_pathTimer.start();

    m_state = State.TURN_ANGLE;
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {

    return PigeonSubsystem.getInstance().getAngle();
    // return Math.IEEEremainder(m_pigeon.getAngle(), 360) *
    // (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    // return _pigeon.getRotation2d().getDegrees();
  }

  public double getRotation() {
    return PigeonSubsystem.getInstance().getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return PigeonSubsystem.getInstance().getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * @param inches
   *               //
   */
  // public void driveDistance(double inches) {
  // while (Math.abs(getAverageEncoderDistance()) <= inches) {
  // tankDrive(.1, .1);

  // }
  // }

  /**
   * @return double
   */
  public double getPitch() {
    return PigeonSubsystem.getInstance().getPitch() - m_pitchOffset;
  }

  /**
   * Stops all the Drive subsytem motors
   */

  public void stop() {
    m_leftMotor.stopMotor();
    m_rightMotor.stopMotor();
  }

  /**
   * @return double
   */
  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(PigeonSubsystem.getInstance().getAngle(), 360);
  }

  /**
   * @return double
   */
  public double getTurnRateCW() {
    // Not negating
    return -PigeonSubsystem.getInstance().getRate();
  }

  /**
   * 
   */
  public void applyBrakesEndGame() {

    if (DriverStation.getMatchTime() < DriveConstants.EndGameSeconds) {
      if (m_brakesOn) {
        setToCoast();
      } else {
        applyBrakes();
      }
    }
  }
 

  public boolean applyBrakes() {

    m_rightMotor.setIdleMode(IdleMode.kBrake);
    m_leftMotor.setIdleMode(IdleMode.kBrake);
    m_rightMotorSlave.setIdleMode(IdleMode.kBrake);
    m_leftMotorSlave.setIdleMode(IdleMode.kBrake);
    m_brakesOn = true;
    SmartDashboard.putString("BRAKES", "ON");
    return true;
  }

  public void setToCoast() {
    m_rightMotor.setIdleMode(IdleMode.kCoast);
    m_leftMotor.setIdleMode(IdleMode.kCoast);
    m_rightMotorSlave.setIdleMode(IdleMode.kCoast);
    m_leftMotorSlave.setIdleMode(IdleMode.kCoast);
    m_brakesOn = false;
    SmartDashboard.putString("BRAKES", "OFF");
  }

  public void endPID() {
    m_distSetpoint = m_defaultSetpoint;
    m_angleSetpoint = m_defaultSetpoint;
    m_distanceProfile = null;
    m_state = m_defaultState;
  }

  public State getState() {
    return m_state;
  }

  public void resetPitch() {
    m_pitchOffset = PigeonSubsystem.getInstance().getPitch();
  }

  @Override
  public void update() {
    // we use brackets in this switch statement to define a local scope
    switch (m_state) {
      // case TANK_DRIVE: {
      // // double adjustedSpeedForward = reverseEnabled ? -speedForward :
      // speedForward;
      // // adjustedSpeedForward = slowModeEnabled ? adjustedSpeedForward *
      // // DRIVE_SLOW_FORWARD_MULT : adjustedSpeedForward;
      // // double adjustedSpeedTurn = slowModeEnabled ? speedTurn *
      // DRIVE_SLOW_TURN_MULT
      // // : speedTurn;

      // // double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(adjustedSpeedForward,
      // // adjustedSpeedTurn);
      // // frontLeftMotor.set(arcadeSpeeds[0]);
      // // frontRightMotor.set(arcadeSpeeds[1]);
      // m_driveDifferential.tankDrive(m_speedLeft, m_speedRight, true);
      // // drivetrain.arcadeDrive(speedForward, speedTurn);
      // break;
      // }
      // case TURN_TRACK: {
      // // kinda misleading var names
      // frontLeftMotor.set(speedForward);
      // frontRightMotor.set(speedTurn);
      // }
      case ARCADE_DRIVE: {
        // double adjustedSpeedForward = reverseEnabled ? -speedForward : speedForward;
        // adjustedSpeedForward = slowModeEnabled ? adjustedSpeedForward *
        // DRIVE_SLOW_FORWARD_MULT : adjustedSpeedForward;
        // double adjustedSpeedTurn = slowModeEnabled ? speedTurn * DRIVE_SLOW_TURN_MULT
        // : speedTurn;

        // apply negative sign to turn speed because WPILib uses left as positive
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(m_arcadeSpeed * MAX_DRIVE_SPEED, 0,
            Math.toRadians(-m_turnAngle * DRIVE_CLOSED_MAX_ROT_TELEOP));
        DifferentialDriveWheelSpeeds wheelSpeeds = m_driveKinematics.toWheelSpeeds(chassisSpeeds);

        m_leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
        m_rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity, DRIVE_VEL_SLOT);
        m_driveDifferential.arcadeDrive(m_arcadeSpeed, 0);
        // for testing output
        // testingTargetLeftSpeed = wheelSpeeds.leftMetersPerSecond;
        // testingTargetRightSpeed = wheelSpeeds.rightMetersPerSecond;
        break;
      }
      case DRIVE_DIST: {
        // if(distSetpoint == defaultSetpoint || angleSetpoint == defaultSetpoint) {
        // state = defaultState;
        // break;
        // }

        // comment this out while initially tuning
        if (m_distancePIDController.atSetpoint()) {
          m_state = m_defaultState;
          m_distSetpoint = m_defaultSetpoint;
          m_angleSetpoint = m_defaultSetpoint;
          break;
        }
        var leftPosition = m_leftEncoder.getPosition();
        double forwardOutput = m_distancePIDController.calculate(leftPosition, m_distSetpoint * -1);
        forwardOutput = MathUtil.clamp(forwardOutput, -DRIVE_DIST_MAX_OUTPUT, DRIVE_DIST_MAX_OUTPUT);
        double turnOutput = (m_angleSetpoint - PigeonSubsystem.getInstance().getAngle()) * DRIVE_DIST_ANGLE_P;

        System.out.println("*********" + forwardOutput);

        m_driveDifferential.arcadeDrive(forwardOutput, turnOutput);
        // double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(forwardOutput, turnOutput);
        // frontLeftMotor.set(arcadeSpeeds[0]);
        // frontRightMotor.set(arcadeSpeeds[1]);
        break;
      }
      case TURN_ANGLE: {
        if (m_angleSetpoint == m_defaultSetpoint) {
          m_state = m_defaultState;
          break;
        }

        // comment this out while initially tuning
        if (m_anglePIDController.atSetpoint() && m_anglePIDController.getVelocityError() < 20) {
          m_state = m_defaultState;
          m_angleSetpoint = m_defaultSetpoint;
          m_driveDifferential.arcadeDrive(0, 0);
          // frontLeftMotor.set(0);
          // frontRightMotor.set(0);
          break;
        }

        if (m_pathTimer.get() > 3) {
          m_state = m_defaultState;
          m_angleSetpoint = m_defaultSetpoint;
          m_driveDifferential.arcadeDrive(0, 0);
          // frontLeftMotor.set(0);
          // frontRightMotor.set(0);
          break;
        }

        double turnOutput = m_anglePIDController.calculate(PigeonSubsystem.getInstance().getAngle(), m_angleSetpoint);
        turnOutput = MathUtil.clamp(turnOutput, -DRIVE_ANGLE_MAX_OUTPUT, DRIVE_ANGLE_MAX_OUTPUT);
        m_driveDifferential.arcadeDrive(0, turnOutput);
        // double[] arcadeSpeeds = ArcadeDrive.arcadeDrive(0, turnOutput);
        // frontLeftMotor.set(arcadeSpeeds[0]);
        // frontRightMotor.set(arcadeSpeeds[1]);
        break;
      }

      default: {
        break;
      }
    }

    updateOdometry();

  }

  public void updateOdometry() {
    // m_poseEstimator.update(Rotation2d.fromDegrees(-PigeonSubsystem.getInstance().getAngle()),
    // m_leftEncoder.getPosition(), m_rightEncoder.getPosition());

    // Optional<EstimatedRobotPose> result =
    // vision.getEstimatedGlobalPose(poseEstimator.getEstimatedPosition());
    // if (result == null)
    // return;

    // if (result.isPresent()) {
    // EstimatedRobotPose camPose = result.get();
    // poseEstimator.addVisionMeasurement(camPose.estimatedPose.toPose2d(),
    // camPose.timestampSeconds);
    // }
  }

  @Override
  public void displayShuffleboard() {

    SmartDashboard.putString("Drive State", m_state.name());
    SmartDashboard.putNumber("Angle Velocity Error", m_anglePIDController.getVelocityError());
    Shuffleboard.getTab("Test").add("Gyro", PigeonSubsystem.getInstance());
  }

  @Override
  public void tuningInit() {
    PigeonSubsystem.getInstance().outputValues();

    SmartDashboard.putNumber("Drive Closed Max Vel", DRIVE_CLOSED_MAX_VEL);
    SmartDashboard.putNumber("Drive Closed Max Rot", DRIVE_CLOSED_MAX_ROT_AUTO);

    SmartDashboard.putNumber("Drive Dist kP", DRIVE_DIST_PID[0]);
    SmartDashboard.putNumber("Drive Dist kI", DRIVE_DIST_PID[1]);
    SmartDashboard.putNumber("Drive Dist kD", DRIVE_DIST_PID[2]);
    SmartDashboard.putNumber("Drive Dist Angle kP", DRIVE_DIST_ANGLE_P);
    SmartDashboard.putNumber("Drive Dist Max Output", DRIVE_DIST_MAX_OUTPUT);

    SmartDashboard.putNumber("Drive Angle kP", DRIVE_ANGLE_PID[0]);
    SmartDashboard.putNumber("Drive Angle kI", DRIVE_ANGLE_PID[1]);
    SmartDashboard.putNumber("Drive Angle kD", DRIVE_ANGLE_PID[2]);
    SmartDashboard.putNumber("Drive Angle Max Output", DRIVE_ANGLE_MAX_OUTPUT);

    SmartDashboard.putNumber("Drive Vel Left kP", DRIVE_VEL_LEFT_P);
    SmartDashboard.putNumber("Drive Vel Left kFF", DRIVE_VEL_LEFT_F);
    SmartDashboard.putNumber("Drive Vel Right kP", DRIVE_VEL_RIGHT_P);
    SmartDashboard.putNumber("Drive Vel Right kFF", DRIVE_VEL_RIGHT_F);

  }

  @Override
  public void tuningPeriodic() {
    PigeonSubsystem.getInstance().outputValues();

    DRIVE_DIST_PID[0] = SmartDashboard.getNumber("Drive Dist kP", DRIVE_DIST_PID[0]);
    DRIVE_DIST_PID[1] = SmartDashboard.getNumber("Drive Dist kI", DRIVE_DIST_PID[1]);
    DRIVE_DIST_PID[2] = SmartDashboard.getNumber("Drive Dist kD", DRIVE_DIST_PID[2]);
    DRIVE_DIST_ANGLE_P = SmartDashboard.getNumber("Drive Dist Angle kP", DRIVE_DIST_ANGLE_P);
    DRIVE_DIST_MAX_OUTPUT = SmartDashboard.getNumber("Drive Dist Max Output", DRIVE_DIST_MAX_OUTPUT);

    if (m_distancePIDController.getP() != DRIVE_DIST_PID[0]) {
      m_distancePIDController.setP(DRIVE_DIST_PID[0]);
    }
    if (m_distancePIDController.getI() != DRIVE_DIST_PID[1]) {
      m_distancePIDController.setI(DRIVE_DIST_PID[1]);
    }
    if (m_distancePIDController.getD() != DRIVE_DIST_PID[2]) {
      m_distancePIDController.setD(DRIVE_DIST_PID[2]);
    }

    DRIVE_ANGLE_PID[0] = SmartDashboard.getNumber("Drive Angle kP", DRIVE_ANGLE_PID[0]);
    DRIVE_ANGLE_PID[1] = SmartDashboard.getNumber("Drive Angle kI", DRIVE_ANGLE_PID[1]);
    DRIVE_ANGLE_PID[2] = SmartDashboard.getNumber("Drive Angle kD", DRIVE_ANGLE_PID[2]);
    DRIVE_ANGLE_MAX_OUTPUT = SmartDashboard.getNumber("Drive Angle Max Output", DRIVE_ANGLE_MAX_OUTPUT);

    if (m_anglePIDController.getP() != DRIVE_ANGLE_PID[0]) {
      m_anglePIDController.setP(DRIVE_ANGLE_PID[0]);
    }
    if (m_anglePIDController.getI() != DRIVE_ANGLE_PID[1]) {
      m_anglePIDController.setI(DRIVE_ANGLE_PID[1]);
    }
    if (m_anglePIDController.getD() != DRIVE_ANGLE_PID[2]) {
      m_anglePIDController.setD(DRIVE_ANGLE_PID[2]);
    }

    DRIVE_VEL_LEFT_P = SmartDashboard.getNumber("Drive Vel Left kP", DRIVE_VEL_LEFT_P);
    DRIVE_VEL_LEFT_F = SmartDashboard.getNumber("Drive Vel Left kFF", DRIVE_VEL_LEFT_F);
    DRIVE_VEL_RIGHT_P = SmartDashboard.getNumber("Drive Vel Right kP", DRIVE_VEL_RIGHT_P);
    DRIVE_VEL_RIGHT_F = SmartDashboard.getNumber("Drive Vel Right kFF", DRIVE_VEL_RIGHT_F);

    if (m_leftPIDController.getP() != DRIVE_VEL_LEFT_P) {
      m_leftPIDController.setP(DRIVE_VEL_LEFT_P);
    }
    if (m_leftPIDController.getFF() != DRIVE_VEL_LEFT_F) {
      m_leftPIDController.setFF(DRIVE_VEL_LEFT_F);
    }
    if (m_rightPIDController.getP() != DRIVE_VEL_RIGHT_P) {
      m_rightPIDController.setP(DRIVE_VEL_RIGHT_P);
    }
    if (m_rightPIDController.getFF() != DRIVE_VEL_RIGHT_F) {
      m_rightPIDController.setFF(DRIVE_VEL_RIGHT_F);
    }

  }

}