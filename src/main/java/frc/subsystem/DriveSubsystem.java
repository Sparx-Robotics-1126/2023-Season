package frc.subsystem;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
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
import frc.robot.Constants;
import frc.robot.IO;
import frc.robot.Constants.DriveConstants;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.ctre.phoenixpro.hardware.Pigeon2;

public class DriveSubsystem extends SubsystemBase  {

    /**
     * The maximum amount of current in amps that should be permitted during motor operation.
     */
    private static final int MAX_CURRENT = 25;
 
    /**
     * The ideal voltage that the motors should attempt to match.
     */
    private static final double NOMINAL_VOLTAGE = 12;

  private DrivesSensorInterface drivesSensors;

  //Put motor initialization here.
  private final CANSparkMax m_rightMotors = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_1, MotorType.kBrushless);
  CANSparkMax rightMotorSlave = new CANSparkMax(IO.DRIVES_RIGHT_MOTOR_2, MotorType.kBrushless);
  
  private final CANSparkMax m_leftMotors = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_1, MotorType.kBrushless);
  CANSparkMax leftMotorSlave = new CANSparkMax(IO.DRIVES_LEFT_MOTOR_2, MotorType.kBrushless);;
  
  //   private final MotorControllerGroup m_leftMotors =
  //     new MotorControllerGroup(
  //         new CANSparkMax(DriveConstants.kLeftMotor1Port, MotorType.kBrushless),
  //         new CANSparkMax(DriveConstants.kLeftMotor2Port, MotorType.kBrushless));

  //         // The motors on the right side of the drive.
  // private final MotorControllerGroup m_rightMotors =
  // new MotorControllerGroup(
  //     new CANSparkMax(DriveConstants.kRightMotor1Port, MotorType.kBrushless),
  //     new CANSparkMax(DriveConstants.kRightMotor2Port, MotorType.kBrushless));

        // The robot's drive
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

// The left-side drive encoder
  private final RelativeEncoder m_leftEncoder = m_leftMotors.getEncoder();
  // // The right-side drive encoder
  private final RelativeEncoder m_rightEncoder =m_rightMotors.getEncoder();


  // The left-side drive encoder
  // private final Encoder m_leftEncoder =
  //     new Encoder(
  //       DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kLeftEncoderPorts[1],
  //         DriveConstants.kLeftEncoderReversed);

  // // The right-side drive encoder
  // private final Encoder m_rightEncoder =
  //     new Encoder(
  //         DriveConstants.kRightEncoderPorts[0],
  //         DriveConstants.kRightEncoderPorts[1],
  //         DriveConstants.kRightEncoderReversed);

           // The gyro sensor
  private final Gyro m_gyro = new Pigeon2(10);

  private final DifferentialDriveOdometry m_odometry;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

    configureMotor(m_rightMotors, rightMotorSlave);
    configureMotor(m_leftMotors, leftMotorSlave);
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotors.setInverted(true);

    // Sets the distance per pulse for the encoders
    // m_leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
    // m_rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

    // resetEncoders();
    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
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
    m_odometry.update(
        m_gyro.getRotation2d(), m_leftEncoder.getPosition(), m_rightEncoder.getPosition());
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
  // public DifferentialDriveWheelSpeeds getWheelSpeeds() {
  //   return new DifferentialDriveWheelSpeeds(m_leftEncoder.get(), m_rightEncoder.getRate());
  // }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  // public void resetOdometry(Pose2d pose) {
  //   resetEncoders();
  //   m_odometry.resetPosition(
  //       m_gyro.getRotation2d(), m_leftEncoder.getDistance(), m_rightEncoder.getDistance(), pose);
  // }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
//   public void arcadeDrive(double fwd, double rot) {
//     m_drive.arcadeDrive(fwd, rot);
//   }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotors.setVoltage(leftVolts);
    m_rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    // m_leftEncoder.reset();
    // m_rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (m_leftEncoder.getPosition() + m_rightEncoder.getPosition()) / 2.0;
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
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

}
