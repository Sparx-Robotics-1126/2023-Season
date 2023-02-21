package frc.robot.subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.drives.DrivesSensorInterface;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {

  private DrivesSensorInterface _drivesSensors;

  // Put motor initialization here.
  CANSparkMax rightMotors;
  CANSparkMax leftMotors;

  // The robot's drive
  public final DifferentialDrive m_driveDifferential;

  private final PIDController m_leftPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final PIDController m_rightPID = new PIDController(DriveConstants.kPDriveVel, 0, 0);
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
      DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);

  // The left-side drive encoder
  private final RelativeEncoder m_leftEncoder;
  // // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder;

  // The gyro sensor
  private final PigeonSubsystem m_pigeon;// = new WPI_Pigeon2(Constants.Pigeon2ID);

  private final DifferentialDriveOdometry m_odometry;
  // private final WPI_Pigeon2 _pigeon2;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PigeonSubsystem pigeon) {
    //stop();
    m_pigeon = pigeon;

    rightMotors = new CANSparkMax(Constants.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax rightMotorSlave = new CANSparkMax(Constants.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);

    leftMotors = new CANSparkMax(Constants.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax leftMotorSlave = new CANSparkMax(Constants.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);

    configureMotor(rightMotors, rightMotorSlave);
    configureMotor(leftMotors, leftMotorSlave);

    m_driveDifferential = new DifferentialDrive(leftMotors, rightMotors);
    //m_driveDifferential.setDeadband(Constants.DEAD_BAND);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // _rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    m_leftEncoder = leftMotors.getEncoder();
    m_rightEncoder = rightMotors.getEncoder();

    m_rightEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);
    m_leftEncoder.setPositionConversionFactor(DriveConstants.kEncoderDistanceConversionFactor);

    //_drivesSensors.addEncoders(m_leftEncoder, m_rightEncoder);
    leftMotors.setInverted(true);
    // Burn settings into Spark MAX flash
    rightMotors.burnFlash();
    leftMotors.burnFlash();

    // Set drive deadband and safety
    m_driveDifferential.setDeadband(0.05);
    m_driveDifferential.setSafetyEnabled(true);

    m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition());

  }

  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
      Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getPosition(),- m_rightEncoder.getPosition());
    SmartDashboard.putNumber("ROBOT_ANGLE", getHeading());

    SmartDashboard.putNumber("LEFT DIST", m_leftEncoder.getPosition());
    SmartDashboard.putNumber("RIGHT DIST", m_rightEncoder.getPosition());
    SmartDashboard.putNumber("AVG DIST", getAverageEncoderDistance());
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
    master.setSmartCurrentLimit(Constants.MAX_CURRENT, 60);
    master.setOpenLoopRampRate(1);

    for (CANSparkMax slave : slaves) {
      slave.restoreFactoryDefaults();
      slave.follow(master);
      slave.setIdleMode(IdleMode.kCoast);
      slave.setSmartCurrentLimit(Constants.MAX_CURRENT, 60);
      slave.setOpenLoopRampRate(1);
    }
  }

  
  /** 
   * @return double
   */
  public double getEncoderMeters() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2 * DriveConstants.kEncoderTick2Meter;
  }

  
  /** 
   * @return Pose2d
   */
  // public void setMotors(double leftSpeed, double rightSpeed) {
  //   leftMotors.set(leftSpeed);
  //   rightMotors.set(-rightSpeed);
  // }


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
        m_pigeon.getRotation2d(), m_leftEncoder.getPosition(),
        m_rightEncoder.getPosition(), pose);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftY  the commanded left output
   * @param rightY the commanded right output
   */
  public void tankDrive(double leftY, double rightY) {
    SmartDashboard.putNumber("LEFTY", leftY);
    SmartDashboard.putNumber("RIGHTY", rightY);
    // SmartDashboard.putNumber("PITCH", _pigeon2subsystem.getPitch());
    // SmartDashboard.putNumber("YAW", _pigeon2subsystem.getYaw());

    m_driveDifferential.tankDrive(leftY, rightY, true);

  }
  
  /** 
   * @param leftVelocitySetpoint
   * @param rightVelocitySetpoint
   */
  public void tankDriveWithFeedforwardPID(double leftVelocitySetpoint, double rightVelocitySetpoint) {
    leftMotors.setVoltage(m_feedforward.calculate(leftVelocitySetpoint)
        + m_leftPID.calculate(m_leftEncoder.getVelocity(), leftVelocitySetpoint));
    rightMotors.setVoltage(m_feedforward.calculate(rightVelocitySetpoint)
        + m_rightPID.calculate(-m_rightEncoder.getVelocity(), rightVelocitySetpoint));
        m_driveDifferential.feed();
}
  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_driveDifferential.arcadeDrive(fwd, rot);
  }

  public void reset() {
    zeroHeading();
    resetEncoders();
    m_odometry.resetPosition( Rotation2d.fromDegrees(getHeading()),m_leftEncoder.getPosition(),
    m_rightEncoder.getPosition(), new Pose2d());

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
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
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_pigeon.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(m_pigeon.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    // return _pigeon.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_pigeon.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  
  /** 
   * @return DrivesSensorInterface
   */
  public DrivesSensorInterface getDriveSenors() {
    return _drivesSensors;
  }

  
  /** 
   * @param inches
   */
  public void driveDistance(double inches) {
    while (Math.abs(getAverageEncoderDistance()) <= inches) {
      tankDrive(.1, .1);

    }
  }

  
  /** 
   * @return double
   */
  public int getPitch() {
    return m_pigeon.getPitch();
  }

  /**
   * Stops all the Drive subsytem motors
   */

  public void stop() {
    leftMotors.stopMotor();
    rightMotors.stopMotor();
  }
  
  /** 
   * @return double
   */
  public double getHeadingCW() {
    // Not negating
    return Math.IEEEremainder(m_pigeon.getAngle(), 360);
  }

  
  /** 
   * @return double
   */
  public double getTurnRateCW() {
    // Not negating
    return -m_pigeon.getRate();
  }

  /**
   * 
   */
  public void applyBrakes(){
    rightMotors.setIdleMode(IdleMode.kBrake);
    leftMotors.setIdleMode(IdleMode.kBrake);
  }

  public void setToCoast(){
    rightMotors.setIdleMode(IdleMode.kCoast);
    leftMotors.setIdleMode(IdleMode.kCoast);
  }

 
}
