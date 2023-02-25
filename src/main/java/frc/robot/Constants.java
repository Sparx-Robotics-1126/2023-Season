package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // MOTORS
  public static final int DRIVES_RIGHT_MOTOR_1 = 25;
  public static final int DRIVES_RIGHT_MOTOR_2 = 27;
  public static final int DRIVES_LEFT_MOTOR_1 = 24;
  public static final int DRIVES_LEFT_MOTOR_2 = 26;
  
  public static final int ELEVATIONS_LEFT_MOTOR = 10;
  public static final int ELEVATIONS_RIGHT_MOTOR = 11;
  public static final int EXTENDERS_LEFT_MOTOR = 7;
  public static final int EXTENDERS_RIGHT_MOTOR = 2;
  
  /**
   * The maximum amount of current in amps that should be permitted during motor
   * operation.
   */
  public static final int MAX_CURRENT = 25;

  /**
   * The ideal voltage that the motors should attempt to match.
   */
  public static final double NOMINAL_VOLTAGE = 12;

  public static final double DEAD_BAND = .5;

  // XBOX Controller
  public static final int XBOX_CONTROLLER_PORT = 0;

  // SENSORS
  public static final I2C.Port I2C_ONBOARD = I2C.Port.kOnboard;
  public static final SerialPort.Port USB_ONBOARD = SerialPort.Port.kUSB;

  public static final int Pigeon2ID = 4;

  public static final int ZeroPigeonYaws = XboxController.Button.kA.value;
  public static final int AddPigeonYaws = XboxController.Button.kB.value;

  public static final class DriveConstants {
    public static final int kLeftMotor1Port = 27;
    public static final int kLeftMotor2Port = 25;
    public static final int kRightMotor1Port = 24;
    public static final int kRightMotor2Port = 26;

    public static final int[] kLeftEncoderPorts = new int[] { 27, 25 };
    public static final int[] kRightEncoderPorts = new int[] { 24, 26 };
    public static final boolean kLeftEncoderReversed = false;
    public static final boolean kRightEncoderReversed = true;

    public static final double kTrackwidthMeters = 0.69;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
        kTrackwidthMeters);

    public static final int kEncoderCPR = 1024;
    public static final double kGearRatio =8.45;// 10.71;
    public static final double kWheelDiameterInches = 6;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
    public static final double kEncoderDistanceConversionFactor = ((double) (Math.PI*kWheelDiameterMeters)/(kGearRatio));



    public static final double kEncoderDistancePerPulse =
        // Assumes the encoders are directly mounted on the wheel shafts
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    public static final double kStabilizationP = 1;
    public static final double kStabilizationI = 0.5;
    public static final double kStabilizationD = 0;

    public static final double kTurnP = 1;
    public static final double kTurnI = 0;
    public static final double kTurnD = 0;

    public static final double kMaxTurnRateDegPerS = 100;
    public static final double kMaxTurnAccelerationDegPerSSquared = 300;

    public static final double kTurnToleranceDeg = 5;
    public static final double kTurnRateToleranceDegPerS = 10; // degrees per second

    public static final double kAutoDriveForwardSpeed = 0.15;
    public static final double kAutoDriveForwardDistance = 1.5;


    public static final double ENCODER_MULTIPLIER = 1.2;

    public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;


     public static final double ksVolts = 0.198;   //0.169
    public static final double kvVoltSecondsPerMeter = 2.86;  //2.24
    public static final double kaVoltSecondsSquaredPerMeter = 0.365;  //0.0435
    public static final double kPDriveVel = 2.24;  //2.4

    public static final boolean kGyroReversed = false;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final int kPitchTimeoutSeconds = 5;
  }
}