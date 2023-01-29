package frc.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.drives.DrivesSensorInterface;
import frc.drives.DrivesSensors;
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenixpro.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase  {

    /**
     * The maximum amount of current in amps that should be permitted during motor operation.
     */
    public static final int MAX_CURRENT = 25;
 
    /**
     * The ideal voltage that the motors should attempt to match.
     */
    public static final double NOMINAL_VOLTAGE = 12;

  // private DrivesSensorInterface _drivesSensors;

  //Put motor initialization here.
  private final CANSparkMax _rightMotors;
  private final CANSparkMax _leftMotors;

        // The robot's drive
  public final DifferentialDrive DriveDifferential;// = new DifferentialDrive(_leftMotors, _rightMotors);

// The left-side drive encoder
  private final RelativeEncoder _leftEncoder; //= _leftMotors.getEncoder();
  // // The right-side drive encoder
  private final RelativeEncoder _rightEncoder;//= _rightMotors.getEncoder();

           // The gyro sensor
 // private final Gyro m_gyro = new WPI_Pigeon2(10);

  //private final DifferentialDriveOdometry m_odometry;
private final  WPI_Pigeon2 _pigeon2;
  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(WPI_Pigeon2 pigeon2 ) {
 //Initialize sensors.
//  _drivesSensors = new DrivesSensors();
_pigeon2 = pigeon2;
    _rightMotors = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax rightMotorSlave = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);
    
    _leftMotors = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
    CANSparkMax leftMotorSlave = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);

    configureMotor(_rightMotors, rightMotorSlave);
    configureMotor(_leftMotors, leftMotorSlave);

    DriveDifferential = new DifferentialDrive(_leftMotors, _rightMotors);
    DriveDifferential.setDeadband(.5);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    //_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    _leftEncoder = _leftMotors.getEncoder();
    _rightEncoder = _rightMotors.getEncoder();

    
    // _drivesSensors.addEncoders(_leftEncoder,_rightEncoder);
    
    // resetEncoders();
    // m_odometry =
    //     new DifferentialDriveOdometry(
    //         m_gyro.getRotation2d(), _leftEncoder.getPosition(), _rightEncoder.getPosition());
  }

  /**
     * Configures motors to follow one controller.
     * @param master The controller to follow.
     * @param slaves The controllers that should follow master.
     */
    private static void configureMotor(CANSparkMax master, CANSparkMax... slaves) 
    {
        master.restoreFactoryDefaults();
        master.set(0);
        master.setIdleMode(IdleMode.kCoast);
        master.enableVoltageCompensation(NOMINAL_VOLTAGE);
        master.setSmartCurrentLimit(MAX_CURRENT);
        master.setOpenLoopRampRate(1);

        for (CANSparkMax slave : slaves) 
        {
            slave.restoreFactoryDefaults();
            slave.follow(master);
            slave.setIdleMode(IdleMode.kCoast);
            slave.setSmartCurrentLimit(MAX_CURRENT);
            slave.setOpenLoopRampRate(1);
        }
    }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    // m_odometry.update(
    //     m_gyro.getRotation2d(), _leftEncoder.getPosition(), _rightEncoder.getPosition());
    //   SmartDashboard.putNumber("ROBOT_ANGLE", m_gyro.getAngle());
      }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return null;
   // return m_odometry.getPoseMeters();
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
    // m_odometry.resetPosition(
    //     m_gyro.getRotation2d(), _leftEncoder.getPosition(), _rightEncoder.getPosition(), pose);
  }


  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftY the commanded left output
   * @param rightY the commanded right output
   */
  public void tankDrive(double leftY, double rightY) {
    SmartDashboard.putNumber("LEFTY", leftY );
    SmartDashboard.putNumber("RIGHTY", rightY);
  //  SmartDashboard.putNumber("PITCH", _pigeon2subsystem.getPitch());
  //  SmartDashboard.putNumber("YAW", _pigeon2subsystem.getYaw());

    DriveDifferential.tankDrive(leftY, rightY,true);

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
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    DriveDifferential.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    // m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return 0.0;
    // return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return 0.0;
    //return -m_gyro.getRate();
  }

}
