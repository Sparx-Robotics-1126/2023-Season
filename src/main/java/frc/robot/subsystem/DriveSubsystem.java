package frc.robot.subsystem;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import frc.drives.DrivesSensorInterface;
import frc.drives.DrivesSensors;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

  private DrivesSensorInterface _drivesSensors;

  // Put motor initialization here.
  private final CANSparkMax _rightMotors;
  private final CANSparkMax _leftMotors;

  // The robot's drive
  public final DifferentialDrive DriveDifferential;

  // The left-side drive encoder
  private final RelativeEncoder _leftEncoder;
  // // The right-side drive encoder
  private final RelativeEncoder _rightEncoder;

  // The gyro sensor
  private final PigeonSubsystem _pigeon;// = new WPI_Pigeon2(Constants.Pigeon2ID);

  private final DifferentialDriveOdometry _odometry;
  // private final WPI_Pigeon2 _pigeon2;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(PigeonSubsystem pigeon) {
    _pigeon = pigeon;
    // Initialize sensors.
    _drivesSensors = new DrivesSensors();
    _rightMotors = new CANSparkMax(Constants.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax rightMotorSlave = new CANSparkMax(Constants.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);

    _leftMotors = new CANSparkMax(Constants.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax leftMotorSlave = new CANSparkMax(Constants.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);

    configureMotor(_rightMotors, rightMotorSlave);
    configureMotor(_leftMotors, leftMotorSlave);

    DriveDifferential = new DifferentialDrive(_leftMotors, _rightMotors);
    DriveDifferential.setDeadband(Constants.DEAD_BAND);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    // _rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    _leftEncoder = _leftMotors.getEncoder();
    _rightEncoder = _rightMotors.getEncoder();

    _drivesSensors.addEncoders(_leftEncoder,_rightEncoder);
    

    // resetEncoders();
    _odometry = new DifferentialDriveOdometry(_pigeon.getRotation2d(),
        _leftEncoder.getPosition(),
        _rightEncoder.getPosition());

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
    master.setOpenLoopRampRate(1);

    for (CANSparkMax slave : slaves) {
      slave.restoreFactoryDefaults();
      slave.follow(master);
      slave.setIdleMode(IdleMode.kCoast);
      slave.setSmartCurrentLimit(Constants.MAX_CURRENT);
      slave.setOpenLoopRampRate(1);
    }
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    _odometry.update(
        _pigeon.getRotation2d(), _leftEncoder.getPosition(),
        _rightEncoder.getPosition());
    SmartDashboard.putNumber("ROBOT_ANGLE", _pigeon.getAngle());

    SmartDashboard.putNumber("ROBOT_POSITION_X", getPose().getX());
    SmartDashboard.putNumber("ROBOT_POSITION_Y", getPose().getY());
    SmartDashboard.putNumber("ROBOT_AVG_DIST", getAverageEncoderDistance());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return _odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(_leftEncoder.getVelocity(), _rightEncoder.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    _odometry.resetPosition(
        _pigeon.getRotation2d(), _leftEncoder.getPosition(),
        _rightEncoder.getPosition(), pose);
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

    DriveDifferential.tankDrive(leftY, rightY, true);

  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    _leftEncoder.setPosition(0);
    _rightEncoder.setPosition(0);
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (_leftEncoder.getPosition() + _rightEncoder.getPosition()) / 2.0;
  }

  /**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public RelativeEncoder getLeftEncoder() {
    return _leftEncoder;
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public RelativeEncoder getRightEncoder() {
    return _rightEncoder;
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more
   * slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    DriveDifferential.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    _pigeon.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return _pigeon.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -_pigeon.getRate();
  }

}
