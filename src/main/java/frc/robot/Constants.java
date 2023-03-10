package frc.robot;

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

    /**
     * The ideal voltage that the motors should attempt to match.
     */
    //
    public static final double NOMINAL_VOLTAGE = 12;
    public static final int MAX_CURRENT = 25;
    public static final double DEAD_BAND = .5;
    public static final int Pigeon2ID = 4;
    public static final int XBOX_DRIVER_CONTROLLER_PORT = 0;
    public static final int XBOX_OPERATOR_CONTROLLER_PORT = 0;

    public static final class DriveConstants {
        // MOTORS
        public static final int DRIVES_RIGHT_MOTOR_1 = 25;
        public static final int DRIVES_RIGHT_MOTOR_2 = 27;
        public static final int DRIVES_LEFT_MOTOR_1 = 24;
        public static final int DRIVES_LEFT_MOTOR_2 = 26;

        public static double MAX_DRIVE_SPEED = 0.9;
        public static double MAX_TRIGGER_SPEED = 0.1;


        public static final int kEncoderCPR = 1024;
        public static final double kGearRatio = 10.75; //8.45;// 10.71;
        public static final double kWheelDiameterInches = 6;
        public static final double kWheelDiameterMeters = Units.inchesToMeters(kWheelDiameterInches);
        public static final double kEncoderDistanceConversionFactor = ( (Math.PI * kWheelDiameterMeters) / (kGearRatio));

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


        public static final double ENCODER_MULTIPLIER = 1.105;

        public static final double kEncoderTick2Meter = 1.0 / 4096.0 * 0.1 * Math.PI;


        public static final double ksVolts = 0.198;   //0.169
        public static final double kvVoltSecondsPerMeter = 2.86;  //2.24
        public static final double kaVoltSecondsSquaredPerMeter = 0.365;  //0.0435
        public static final double kPDriveVel = 2.24;  //2.4

        public static final boolean kGyroReversed = false;
        
        public static final int EndGameSeconds = 0;
    }

    public static final class AcquisitionConstants {
        public static final int ELEVATIONS_LEFT_MOTOR = 10;
        public static final int ELEVATIONS_RIGHT_MOTOR = 7;
        public static final int EXTENDERS_LEFT_MOTOR = 8;
        public static final int EXTENDERS_RIGHT_MOTOR = 9;
        
        public static final int X_MAX_METERS = 0;
        public static final int Y_MAX_METERS = 0;
        public static final int POSITION_EPSILON_METERS = 0;
        
        public static final int MOTOR_SPEED = 0;
        
        public static final int TICKS_TO_METERS = 0;

        public static final int MIN_PRESSURE = 0;
        public static final int MAX_PRESSURE = 0;

    }

    public static final class ChooserOptions{
        static final String kAutoShort = "Short";
         static final String kAutoLong = "Long";
         static final String kDriveMeasure = "Drive Measure";
         static final String kScoreCommunity = "Score and Leave Community";
    }

    public static final class AutoConstants {

        public static final int kPitchTimeoutSeconds = 5;
    }
}